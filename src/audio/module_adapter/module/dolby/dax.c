// SPDX-License-Identifier: BSD-3-Clause
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE
//
// Copyright(c) 2025 Dolby Laboratories. All rights reserved.
//
// Author: Jun Lai <jun.lai@dolby.com>
//

#include <rtos/atomic.h>
#include <rtos/init.h>
#include <sof/audio/data_blob.h>
#include <sof/audio/module_adapter/module/generic.h>
#include <sof/compiler_attributes.h>

#include "dax.h"

/*
 * DAX explicitly does not support user mode: it must always run in a
 * kernel-mode DP thread for following reasons:
 *   - shared state is serialised with k_spin_lock(), which requires privileged
 *     interrupt masking and is therefore not usable from a user thread,
 *   - the cross-instance DAX instance manager is a kernel-only static living in
 *     base firmware. it is not mapped into a DP module memory domain,
 *   - not every helper it calls from module context (for instance comp_data_blob_set)
 *     is exported as a syscall.
 *   - a explicit persist/scratch/IO heap buffer size is required for user mode (default is 20KB)
 */
#if CONFIG_USERSPACE && !CONFIG_SOF_USERSPACE_PROXY
#error "Dolby DAX is not user-mode safe"
#endif

LOG_MODULE_REGISTER(dolby_dax_audio_processing, CONFIG_SOF_LOG_LEVEL);
SOF_DEFINE_REG_UUID(dolby_dax_audio_processing);

#define DAX_PROFILE_MASK 0x2
#define DAX_DEVICE_MASK 0x4
#define DAX_CP_MASK 0x8
#define DAX_VOLUME_MASK 0x10
#define DAX_CTC_MASK 0x20
#define DAX_TUNING_FILE_MASK 0x40
#define DAX_PROCESSING_MASK 0x10000
#define DAX_RESET_MASK 0x20000
#define DAX_FREE_MASK 0x40000

#define DAX_SWITCH_ENABLE_CONTROL_ID 0
#define DAX_SWITCH_CP_CONTROL_ID 1
#define DAX_SWITCH_CTC_CONTROL_ID 2
#define DAX_ENUM_PROFILE_CONTROL_ID 0
#define DAX_ENUM_DEVICE_CONTROL_ID 1

#define IPC4_DAX_MODULE_CFG_PARAM_BLOB 1

/*
 * Number of DAX blocks of latency introduced by the algorithm. The algorithm
 * outputs the audio that was fed DAX_LATENCY_BLOCKS blocks earlier, therefore
 * it has to be primed with that many blocks before its output can be used.
 */
#ifndef DAX_LATENCY_BLOCKS
#define DAX_LATENCY_BLOCKS 1
#endif

/*
 * Length of a single fade-out or fade-in ramp, in frames. Every path switch
 * performs a fade-out of the active path followed by a fade-in of the
 * destination path.
 */
#ifndef DAX_XFADE_FRAMES
#define DAX_XFADE_FRAMES 256
#endif

/* Q31 representation of a unity gain. */
#define DAX_GAIN_FULL INT32_MAX

enum dax_flag_opt_mode {
	DAX_FLAG_READ = 0,
	DAX_FLAG_SET,
	DAX_FLAG_CLEAR,
	DAX_FLAG_READ_AND_CLEAR,
};

static int32_t flag_process(struct dax_adapter_data *adapter_data,
			    uint32_t flag,
			    enum dax_flag_opt_mode opt_mode)
{
#ifdef __ZEPHYR__
	int32_t bit = ffs(flag) - 1;

	switch (opt_mode) {
	case DAX_FLAG_READ:
		return atomic_test_bit(&adapter_data->proc_flags, bit);
	case DAX_FLAG_SET:
		atomic_set_bit(&adapter_data->proc_flags, bit);
		break;
	case DAX_FLAG_CLEAR:
		atomic_clear_bit(&adapter_data->proc_flags, bit);
		break;
	case DAX_FLAG_READ_AND_CLEAR:
		return atomic_test_and_clear_bit(&adapter_data->proc_flags, bit);
	default:
		break;
	}
#else
	/* Non-Zephyr builds run single-threaded (no DP mode), there is no synchronous problem */
	int32_t old_flags = atomic_read(&adapter_data->proc_flags);

	switch (opt_mode) {
	case DAX_FLAG_READ:
		return (old_flags & flag) != 0;
	case DAX_FLAG_SET:
		atomic_set(&adapter_data->proc_flags, old_flags | flag);
		break;
	case DAX_FLAG_CLEAR:
		atomic_set(&adapter_data->proc_flags, old_flags & ~flag);
		break;
	case DAX_FLAG_READ_AND_CLEAR:
		atomic_set(&adapter_data->proc_flags, old_flags & ~flag);
		return (old_flags & flag) != 0;
	default:
		break;
	}
#endif
	return 0;
}

static int itostr(int num, char *str)
{
	int index = 0, digit_count = 0;
	int temp;

	if (num < 0) {
		str[0] = '-';
		index = 1;
		num = -num;
	}

	if (num == 0) {
		str[index] = '0';
		str[index + 1] = '\0';
		return index + 1;
	}

	temp = num;
	while (temp > 0) {
		temp /= 10;
		digit_count++;
	}

	temp = index + digit_count - 1;
	while (num > 0) {
		str[temp] = (num % 10) + '0';
		num /= 10;
		temp--;
	}

	str[index + digit_count] = '\0';
	return index + digit_count;
}

static void get_params_str(const void *val, uint32_t val_sz,
			   char *params_str, size_t params_str_sz)
{
	const int32_t *param_val = (const int32_t *)val;
	const uint32_t param_sz = val_sz >> 2;
	uint32_t offset = 0;

	if (params_str_sz == 0)
		return;
	params_str[0] = '\0';

	/* Assume each value is an int32_t with maximum 16 characters when converted to string */
	for (uint32_t i = 0; i < param_sz && offset + 16 < params_str_sz; i++) {
		offset += itostr(param_val[i], params_str + offset);
		params_str[offset] = ',';
		offset++;
		params_str[offset] = '\0';
	}
}

static int sof_to_dax_frame_fmt(enum sof_ipc_frame sof_frame_fmt)
{
	switch (sof_frame_fmt) {
	case SOF_IPC_FRAME_S16_LE:
		return DAX_FMT_SHORT_16;
	case SOF_IPC_FRAME_S32_LE:
		return DAX_FMT_INT;
	case SOF_IPC_FRAME_FLOAT:
		return DAX_FMT_FLOAT;
	default:
		return DAX_FMT_UNSUPPORTED;
	}
}

static int sof_to_dax_sample_rate(uint32_t rate)
{
	switch (rate) {
	case 48000:
		return rate;
	default:
		return DAX_RATE_UNSUPPORTED;
	}
}

static int sof_to_dax_channels(uint32_t channels)
{
	switch (channels) {
	case 2:
	case 6 /* 5.1 */:
	case 8 /* 7.1 */:
		return channels;
	default:
		return DAX_CHANNLES_UNSUPPORTED;
	}
}

static int sof_to_dax_buffer_layout(enum sof_ipc_buffer_format sof_buf_fmt)
{
	switch (sof_buf_fmt) {
	case SOF_IPC_BUFFER_INTERLEAVED:
		return DAX_BUFFER_LAYOUT_INTERLEAVED;
	case SOF_IPC_BUFFER_NONINTERLEAVED:
		return DAX_BUFFER_LAYOUT_NONINTERLEAVED;
	default:
		return DAX_BUFFER_LAYOUT_UNSUPPORTED;
	}
}

void dax_buffer_release(struct processing_module *mod, struct dax_buffer *dax_buff)
{
	if (dax_buff->addr) {
		mod_free(mod, dax_buff->addr);
		dax_buff->addr = NULL;
	}
	dax_buff->size = 0;
	dax_buff->avail = 0;
	dax_buff->free = 0;
}

int dax_buffer_alloc(struct processing_module *mod,
		     struct dax_buffer *dax_buff, uint32_t bytes)
{
	dax_buffer_release(mod, dax_buff);
	dax_buff->addr = mod_balloc(mod, bytes);
	if (!dax_buff->addr)
		dax_buff->addr = mod_zalloc(mod, bytes);
	if (!dax_buff->addr)
		return -ENOMEM;

	dax_buff->size = bytes;
	dax_buff->avail = 0;
	dax_buff->free = bytes;
	return 0;
}

/* After reading from buffer */
static void dax_buffer_consume(struct dax_buffer *dax_buff, uint32_t bytes)
{
	uint8_t *buf = (uint8_t *)dax_buff->addr;
	uint32_t copy_bytes;

	if (!bytes)
		return;

	bytes = MIN(bytes, dax_buff->avail);
	copy_bytes = dax_buff->avail - bytes;
	for (int i = 0; i < copy_bytes; i++)
		buf[i] = buf[bytes + i];
	dax_buff->avail = copy_bytes;
	dax_buff->free = dax_buff->size - dax_buff->avail;
}

/* After writing to buffer */
static void dax_buffer_produce(struct dax_buffer *dax_buff, uint32_t bytes)
{
	dax_buff->avail += bytes;
	dax_buff->avail = MIN(dax_buff->avail, dax_buff->size);
	dax_buff->free = dax_buff->size - dax_buff->avail;
}

/**
 * @brief Drop all the data pending in a DAX buffer.
 *
 * Data of a DAX buffer is always kept contiguous at the beginning of the
 * allocation (see dax_buffer_consume()), hence resetting the pending length is
 * enough to discard it.
 *
 * @param[in,out] dax_buff Pointer to the DAX buffer to reset.
 */
static void dax_buffer_discard(struct dax_buffer *dax_buff)
{
	dax_buff->avail = 0;
	dax_buff->free = dax_buff->size;
}

/**
 * @brief Advance the crossfade ramp by one audio frame.
 *
 * @param[in,out] xf Crossfade context.
 */
static void dax_xfade_next_frame(struct dax_crossfade *xf)
{
	int32_t gain;

	if (!xf->fade_left)
		return;

	xf->fade_left--;
	if (!xf->fade_left) {
		/* Land exactly on the target gain to avoid a rounding residue */
		xf->gain = xf->gain_step > 0 ? DAX_GAIN_FULL : 0;
		return;
	}

	gain = xf->gain + xf->gain_step;
	xf->gain = MAX(MIN(gain, DAX_GAIN_FULL), 0);
}

/**
 * @brief Apply a constant Q31 gain to audio frames.
 *
 * @param[out] dst Destination buffer.
 * @param[in] src Source buffer.
 * @param[in] frames Number of audio frames to scale.
 * @param[in] channels Number of channels per frame.
 * @param[in] fmt Sample format.
 * @param[in] gain Q31 gain to apply.
 */
static void dax_apply_gain(void *dst, const void *src, uint32_t frames,
			   uint32_t channels, enum sof_ipc_frame fmt, int32_t gain)
{
	uint32_t i, samples = frames * channels;

	switch (fmt) {
	case SOF_IPC_FRAME_S16_LE: {
		int16_t *d = dst;
		const int16_t *s = src;

		for (i = 0; i < samples; i++) {
			int64_t v = (int64_t)(*s++) * gain;

			*d++ = (int16_t)((v + (1LL << 30)) >> 31);
		}
		break;
	}
	case SOF_IPC_FRAME_S32_LE: {
		int32_t *d = dst;
		const int32_t *s = src;

		for (i = 0; i < samples; i++) {
			int64_t v = (int64_t)(*s++) * gain;

			*d++ = (int32_t)((v + (1LL << 30)) >> 31);
		}
		break;
	}
	case SOF_IPC_FRAME_FLOAT: {
		float *d = dst;
		const float *s = src;
		float g = (float)gain * (1.0f / 2147483648.0f);

		for (i = 0; i < samples; i++)
			*d++ = *s++ * g;
		break;
	}
	default:
		/* The format is validated by check_media_format() */
		break;
	}
}

/**
 * @brief Copy audio frames applying the crossfade ramp.
 *
 * The first frames are scaled with a gain that changes by one step per frame.
 * Any remaining frames are played at the gain reached at the end of the ramp.
 *
 * @param[out] dst Destination buffer.
 * @param[in] src Source buffer.
 * @param[in] frames Number of audio frames to copy.
 * @param[in] channels Number of channels per frame.
 * @param[in] fmt Sample format.
 * @param[in] frame_bytes Size of one audio frame in bytes.
 * @param[in,out] xf Crossfade context.
 */
static void dax_xfade_scale(void *dst, const void *src, uint32_t frames,
			    uint32_t channels, enum sof_ipc_frame fmt,
			    uint32_t frame_bytes, struct dax_crossfade *xf)
{
	uint32_t ramp = MIN(frames, xf->fade_left);
	uint32_t i, c;

	if (ramp) {
		switch (fmt) {
		case SOF_IPC_FRAME_S16_LE: {
			int16_t *d = dst;
			const int16_t *s = src;

			for (i = 0; i < ramp; i++) {
				for (c = 0; c < channels; c++) {
					int64_t v = (int64_t)(*s++) * xf->gain;

					*d++ = (int16_t)((v + (1LL << 30)) >> 31);
				}
				dax_xfade_next_frame(xf);
			}
			break;
		}
		case SOF_IPC_FRAME_S32_LE: {
			int32_t *d = dst;
			const int32_t *s = src;

			for (i = 0; i < ramp; i++) {
				for (c = 0; c < channels; c++) {
					int64_t v = (int64_t)(*s++) * xf->gain;

					*d++ = (int32_t)((v + (1LL << 30)) >> 31);
				}
				dax_xfade_next_frame(xf);
			}
			break;
		}
		case SOF_IPC_FRAME_FLOAT: {
			float *d = dst;
			const float *s = src;

			for (i = 0; i < ramp; i++) {
				float g = (float)xf->gain * (1.0f / 2147483648.0f);

				for (c = 0; c < channels; c++)
					*d++ = *s++ * g;
				dax_xfade_next_frame(xf);
			}
			break;
		}
		default:
			/* An unsupported format must not stall the state machine */
			memcpy(dst, src, (size_t)ramp * frame_bytes);
			for (i = 0; i < ramp; i++)
				dax_xfade_next_frame(xf);
			break;
		}
	}

	if (frames == ramp)
		return;

	dst = (uint8_t *)dst + (size_t)ramp * frame_bytes;
	src = (const uint8_t *)src + (size_t)ramp * frame_bytes;
	if (xf->gain == DAX_GAIN_FULL)
		memcpy(dst, src, (size_t)(frames - ramp) * frame_bytes);
	else if (!xf->gain)
		memset(dst, 0, (size_t)(frames - ramp) * frame_bytes);
	else
		dax_apply_gain(dst, src, frames - ramp, channels, fmt, xf->gain);
}

/**
 * @brief Write audio data from a linear buffer into a sink.
 *
 * @param[in] sink Destination sink.
 * @param[in] src Linear source buffer.
 * @param[in] avail Number of bytes available in @p src.
 * @param[in] frame_bytes Size of one audio frame in bytes.
 * @param[in] channels Number of channels per frame.
 * @param[in] fmt Sample format.
 * @param[in,out] xf Crossfade context, or NULL to write at unity gain.
 *
 * @return Number of bytes written to the sink.
 */
static uint32_t dax_write_sink(struct sof_sink *sink, const void *src, uint32_t avail,
			       uint32_t frame_bytes, uint32_t channels,
			       enum sof_ipc_frame fmt, struct dax_crossfade *xf)
{
	const uint8_t *src8 = src;
	uint32_t bytes, done = 0;
	uint8_t *dst, *bufstart;
	size_t bufsz;

	if (!frame_bytes)
		return 0;

	bytes = MIN(avail, (uint32_t)sink_get_free_size(sink));
	bytes -= bytes % frame_bytes;
	if (!bytes)
		return 0;

	if (sink_get_buffer(sink, bytes, (void **)&dst, (void **)&bufstart, &bufsz) < 0)
		return 0;

	while (done < bytes) {
		uint8_t *end = bufstart + bufsz;
		uint32_t chunk;

		if (dst == end)
			dst = bufstart;

		chunk = MIN(bytes - done, (uint32_t)(end - dst));
		if (xf)
			dax_xfade_scale(dst, src8 + done, chunk / frame_bytes,
					channels, fmt, frame_bytes, xf);
		else
			memcpy(dst, src8 + done, chunk);
		done += chunk;
		dst = bufstart;
	}

	sink_commit_buffer(sink, bytes);
	return bytes;
}

/**
 * @brief Move pending data of a DAX buffer into a sink.
 *
 * @param[in] sink Destination sink.
 * @param[in,out] dax_buff DAX buffer to read from.
 * @param[in] frame_bytes Size of one audio frame in bytes.
 * @param[in] channels Number of channels per frame.
 * @param[in] fmt Sample format.
 * @param[in,out] xf Crossfade context, or NULL to write at unity gain.
 *
 * @return Number of bytes written and consumed.
 */
static uint32_t dax_buffer_to_sink(struct sof_sink *sink, struct dax_buffer *dax_buff,
				   uint32_t frame_bytes, uint32_t channels,
				   enum sof_ipc_frame fmt, struct dax_crossfade *xf)
{
	uint32_t bytes;

	bytes = dax_write_sink(sink, dax_buff->addr, dax_buff->avail,
			       frame_bytes, channels, fmt, xf);
	if (bytes)
		dax_buffer_consume(dax_buff, bytes);
	return bytes;
}

/**
 * @brief Copy data from a source into one or two DAX buffers.
 *
 * The same data can be duplicated into two buffers, which is required when the
 * passthrough signal has to be output while the algorithm is also being fed.
 *
 * @param[in] source Source to read from.
 * @param[in,out] first First destination buffer, or NULL.
 * @param[in,out] second Second destination buffer, or NULL.
 * @param[in] max_bytes Maximum number of bytes to read.
 *
 * @return Number of bytes read from the source.
 */
static uint32_t dax_source_to_buffers(struct sof_source *source,
				      struct dax_buffer *first,
				      struct dax_buffer *second,
				      uint32_t max_bytes)
{
	const void *data, *bufstart;
	size_t bufsz;
	uint32_t bytes;
	uint8_t *dax_buf;

	bytes = MIN((uint32_t)source_get_data_available(source), max_bytes);
	if (!bytes)
		return 0;
	if (source_get_data(source, bytes, &data, &bufstart, &bufsz) < 0)
		return 0;

	for (int i = 0; i < 2; i++) {
		struct dax_buffer *dax_buff = i ? second : first;

		if (!dax_buff)
			continue;

		dax_buf = dax_buff->addr;
		cir_buf_copy(data, bufstart, (const uint8_t *)bufstart + bufsz,
			     dax_buf + dax_buff->avail, dax_buf, dax_buf + dax_buff->size,
			     bytes);
		dax_buffer_produce(dax_buff, bytes);
	}

	source_release_data(source, bytes);
	return bytes;
}

/**
 * @brief Run one DAX block through the algorithm.
 *
 * A block is only submitted when a full input period is available, so that the
 * algorithm always works on complete periods.
 *
 * @param[in] mod Pointer to the processing module.
 *
 * @return Number of bytes processed, 0 when no full block is available or the
 *	   algorithm produced nothing, negative error code on failure.
 */
static int dax_run_process(struct processing_module *mod)
{
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);
	struct sof_dax *dax_ctx = &adapter_data->dax_ctx;
	struct dax_crossfade *xf = &adapter_data->xfade;
	int32_t processed;

	if (!dax_ctx->p_dax || !dax_ctx->enable || !xf->input_period_bytes)
		return 0;

	if (dax_ctx->input_buffer.avail < xf->input_period_bytes)
		return 0;

	processed = dax_process(dax_ctx);
	if (processed < 0) {
		comp_err(mod->dev, "dax_process() failed, ret %d", processed);
		return processed;
	}
	if (!processed)
		return 0;

	if ((uint32_t)processed > dax_ctx->output_buffer.free)
		comp_warn(mod->dev, "output buffer overflow, %u bytes dropped",
			  (uint32_t)processed - dax_ctx->output_buffer.free);

	dax_buffer_consume(&dax_ctx->input_buffer, processed);
	dax_buffer_produce(&dax_ctx->output_buffer, processed);
	return processed;
}

static int get_next_param(struct comp_dev *dev, void *data, uint32_t data_size,
			      uint32_t *pos, struct module_param **param)
{
	const uint32_t header_size = sizeof(struct module_param);
	struct module_param *p = (struct module_param *)((uint8_t *)data + *pos);
	uint32_t remain = data_size - *pos;

	if (remain < header_size || p->size < header_size || p->size > remain ||
	    (p->size & 0x03) != 0) {
		comp_err(dev, "invalid param at pos %u, remain %u", *pos, remain);
		return -EINVAL;
	}

	*param = p;
	*pos += p->size;
	return 0;
}

static int dax_cache_config(struct processing_module *mod,
			    uint32_t id, void *value, uint32_t size)
{
	int ret = 0;
	struct comp_dev *dev = mod->dev;
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);
	struct dax_cache_params *cache_params = &adapter_data->cache_params;
	struct dax_buffer new_tuning_buf = { 0 };
	struct dax_buffer old_tuning_buf = { 0 };
	k_spinlock_key_t key;
	int32_t tmp_val;

	switch (id) {
	case DAX_PARAM_ID_TUNING_FILE:
	{
		struct module_param *param;
		uint32_t pos = 0;

		/* The blob carries the tuning file as its first module_param record */
		ret = get_next_param(dev, value, size, &pos, &param);
		if (ret != 0)
			return ret;
		if (param->id != DAX_PARAM_ID_TUNING_FILE ||
		    param->size <= sizeof(struct module_param)) {
			comp_err(dev, "invalid tuning file param %#x, size %u",
				 param->id, param->size);
			return -EINVAL;
		}
		size = param->size - sizeof(struct module_param);

		ret = dax_buffer_alloc(mod, &new_tuning_buf, size);
		if (ret != 0) {
			comp_err(dev, "allocate %u bytes failed for tuning file, ret %d", size,
				 ret);
			break;
		}
		ret = memcpy_s(new_tuning_buf.addr, new_tuning_buf.free, param->data, size);
		if (ret != 0) {
			comp_err(dev, "copy tuning file failed, ret %d", ret);
			dax_buffer_release(mod, &new_tuning_buf);
			break;
		}
		key = k_spin_lock(&adapter_data->lock);
		old_tuning_buf = cache_params->tuning_file;
		cache_params->tuning_file = new_tuning_buf;
		k_spin_unlock(&adapter_data->lock, key);
		dax_buffer_release(mod, &old_tuning_buf);
		flag_process(adapter_data, DAX_TUNING_FILE_MASK, DAX_FLAG_SET);
		comp_info(dev, "allocated: tuning %p, size %u", new_tuning_buf.addr,
			  new_tuning_buf.size);
		break;
	}
	case DAX_PARAM_ID_ENABLE:
		tmp_val = *((int32_t *)value);
		tmp_val = !!tmp_val;
		if (cache_params->enable != tmp_val)
			cache_params->enable = tmp_val;
		break;
	case DAX_PARAM_ID_ABSOLUTE_VOLUME:
		tmp_val = *((int32_t *)value);
		if (cache_params->volume != tmp_val) {
			cache_params->volume = tmp_val;
			flag_process(adapter_data, DAX_VOLUME_MASK, DAX_FLAG_SET);
		}
		break;
	case DAX_PARAM_ID_OUT_DEVICE:
		tmp_val = *((int32_t *)value);
		if (cache_params->out_device != tmp_val) {
			cache_params->out_device = tmp_val;
			flag_process(adapter_data, DAX_DEVICE_MASK, DAX_FLAG_SET);
		}
		break;
	case DAX_PARAM_ID_PROFILE:
		tmp_val = *((int32_t *)value);
		if (cache_params->profile != tmp_val) {
			cache_params->profile = tmp_val;
			flag_process(adapter_data, DAX_PROFILE_MASK, DAX_FLAG_SET);
		}
		break;
	case DAX_PARAM_ID_CP_ENABLE:
		tmp_val = *((int32_t *)value);
		tmp_val = !!tmp_val;
		if (cache_params->content_processing_enable != tmp_val) {
			cache_params->content_processing_enable = tmp_val;
			flag_process(adapter_data, DAX_CP_MASK, DAX_FLAG_SET);
		}
		break;
	case DAX_PARAM_ID_CTC_ENABLE:
		tmp_val = *((int32_t *)value);
		tmp_val = !!tmp_val;
		if (cache_params->ctc_enable != tmp_val) {
			cache_params->ctc_enable = tmp_val;
			flag_process(adapter_data, DAX_CTC_MASK, DAX_FLAG_SET);
		}
		break;
	default:
		comp_err(dev, "unsupported param %#x", id);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*
 * This function applies parameters to the DAX instance and only be
 * called from the sof_dax_process() thread.
 */
static int apply_blob_params(struct processing_module *mod, void *data, uint32_t data_size,
			     uint32_t depth)
{
	struct comp_dev *dev = mod->dev;
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);
	struct sof_dax *dax_ctx = &adapter_data->dax_ctx;
	struct module_param *param;
	uint32_t param_data_size;
	int ret;

	for (uint32_t pos = 0; pos < data_size;) {
		ret = get_next_param(dev, data, data_size, &pos, &param);
		if (ret != 0)
			return ret;

		param_data_size = param->size - sizeof(struct module_param);
		if (param_data_size == 0)
			continue;

		if (param->id == DAX_PARAM_ID_ENDPOINT) {
			/* Endpoint record: data[0] is the endpoint, followed by nested params */
			if (param_data_size < sizeof(int32_t)) {
				comp_err(dev, "invalid endpoint param size %u", param_data_size);
				return -EINVAL;
			}
			if (dax_ctx->endpoint != param->data[0])
				continue;

			if (depth >= 1) {
				comp_err(dev, "endpoint nesting exceeds");
				return -EINVAL;
			}

			ret = apply_blob_params(mod, &param->data[1],
						param_data_size - sizeof(int32_t), depth + 1);
			comp_info(dev, "switched to endpoint %d, ret %d", dax_ctx->endpoint, ret);
			continue;
		}

		ret = dax_set_param(param->id, param->data, param_data_size, dax_ctx);
		get_params_str(param->data, param_data_size, adapter_data->params_str,
			       sizeof(adapter_data->params_str));
		comp_info(dev, "dax_set_param: ret %d, id %#x, size %u, value %s",
			  ret, param->id, param_data_size >> 2, adapter_data->params_str);
	}

	return 0;
}

static bool is_enabled(struct processing_module *mod)
{
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);
	struct sof_dax *dax_ctx = &adapter_data->dax_ctx;

	return dax_ctx->enable && dax_ctx->p_dax;
}

static int set_tuning_file(struct processing_module *mod)
{
	int ret = -EINVAL;
	struct comp_dev *dev = mod->dev;
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);
	struct dax_cache_params *cache_params = &adapter_data->cache_params;
	struct sof_dax *dax_ctx = &adapter_data->dax_ctx;
	struct dax_buffer old_tuning_buf = {0};
	k_spinlock_key_t key;

	key = k_spin_lock(&adapter_data->lock);
	if (cache_params->tuning_file.addr && cache_params->tuning_file.size > 0) {
		old_tuning_buf = dax_ctx->tuning_file_buffer;
		dax_ctx->tuning_file_buffer = cache_params->tuning_file;
		cache_params->tuning_file = (struct dax_buffer){0};
		ret = 0;
	}
	k_spin_unlock(&adapter_data->lock, key);

	dax_buffer_release(mod, &old_tuning_buf);

	comp_info(dev, "apply tuning %p, ret %d", dax_ctx->tuning_file_buffer.addr, ret);
	return ret;
}

static int set_enable(struct processing_module *mod, int32_t enable)
{
	int ret;
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);
	struct sof_dax *dax_ctx = &adapter_data->dax_ctx;
	int32_t old_enable = dax_ctx->enable;

	dax_ctx->enable = enable;
	if (!dax_ctx->p_dax)
		return 0;

	ret = dax_set_enable(enable, dax_ctx);
	if (ret)
		dax_ctx->enable = old_enable;
	comp_info(mod->dev, "set dax enable %d, ret %d", enable, ret);
	return ret;
}

static int set_volume(struct processing_module *mod, int32_t abs_volume)
{
	int ret;
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);
	struct sof_dax *dax_ctx = &adapter_data->dax_ctx;
	int32_t old_volume = dax_ctx->volume;

	dax_ctx->volume = abs_volume;
	ret = dax_set_volume(abs_volume, dax_ctx);
	if (ret)
		dax_ctx->volume = old_volume;
	comp_info(mod->dev, "set volume %d, ret %d", abs_volume, ret);
	return ret;
}

static int set_device(struct processing_module *mod, int32_t out_device)
{
	int ret;
	int tuning_device_ret = -EINVAL;
	struct comp_dev *dev = mod->dev;
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);
	struct sof_dax *dax_ctx = &adapter_data->dax_ctx;
	int32_t old_out_device = dax_ctx->out_device;
	uint32_t params_sz = 0;
	void *params;

	dax_ctx->out_device = out_device;
	ret = dax_set_device(out_device, dax_ctx);
	if (ret)
		dax_ctx->out_device = old_out_device;
	comp_info(dev, "set device %d, ret %d", out_device, ret);

	params = dax_find_params(DAX_PARAM_ID_TUNING_DEVICE, dax_ctx->tuning_device,
				 &params_sz, dax_ctx);
	if (params)
		tuning_device_ret = apply_blob_params(mod, params, params_sz, 0);
	comp_info(dev, "switched to tuning device %d, ret %d",
		  dax_ctx->tuning_device, tuning_device_ret);

	return ret;
}

static int set_crosstalk_cancellation_enable(struct processing_module *mod, int32_t enable)
{
	int ret;
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);
	struct sof_dax *dax_ctx = &adapter_data->dax_ctx;
	int32_t old_ctc_enable = dax_ctx->ctc_enable;

	dax_ctx->ctc_enable = enable;
	ret = dax_set_ctc_enable(enable, dax_ctx);
	if (ret)
		dax_ctx->ctc_enable = old_ctc_enable;
	comp_info(mod->dev, "set ctc enable %d, ret %d", enable, ret);
	return ret;
}

static int set_profile(struct processing_module *mod, int32_t profile_id)
{
	int ret = -EINVAL;
	struct comp_dev *dev = mod->dev;
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);
	struct sof_dax *dax_ctx = &adapter_data->dax_ctx;
	uint32_t params_sz = 0;
	void *params;

	params = dax_find_params(DAX_PARAM_ID_PROFILE, profile_id, &params_sz, dax_ctx);
	if (params)
		ret = apply_blob_params(mod, params, params_sz, 0);
	if (ret == 0)
		dax_ctx->profile = profile_id;
	comp_info(dev, "switched to profile %d, ret %d", profile_id, ret);
	return ret;
}

static int set_content_processing_enable(struct processing_module *mod, int32_t enable)
{
	int ret = -EINVAL;
	struct comp_dev *dev = mod->dev;
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);
	struct sof_dax *dax_ctx = &adapter_data->dax_ctx;
	uint32_t params_sz = 0;
	void *params;

	params = dax_find_params(DAX_PARAM_ID_CP_ENABLE, enable, &params_sz, dax_ctx);
	if (params)
		ret = apply_blob_params(mod, params, params_sz, 0);
	if (ret == 0)
		dax_ctx->content_processing_enable = enable;
	comp_info(dev, "set content processing enable %d, ret %d", enable, ret);
	return ret;
}

static void dax_apply_configs(struct processing_module *mod)
{
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);
	struct dax_cache_params *cache_params = &adapter_data->cache_params;

	/* The enable state is handled by the path state machine, it decides when
	 * the algorithm can actually be switched on or off.
	 */
	if (!is_enabled(mod))
		return;

	if (flag_process(adapter_data, DAX_TUNING_FILE_MASK, DAX_FLAG_READ_AND_CLEAR)) {
		set_tuning_file(mod);
		flag_process(adapter_data, DAX_DEVICE_MASK, DAX_FLAG_SET);
		flag_process(adapter_data, DAX_VOLUME_MASK, DAX_FLAG_SET);
	}

	/* Apply every pending change in a single pass, in the order required by
	 * the DAX configuration dependencies.
	 */
	if (flag_process(adapter_data, DAX_DEVICE_MASK, DAX_FLAG_READ_AND_CLEAR)) {
		set_device(mod, cache_params->out_device);
		flag_process(adapter_data, DAX_PROFILE_MASK, DAX_FLAG_SET);
	}
	if (flag_process(adapter_data, DAX_CTC_MASK, DAX_FLAG_READ_AND_CLEAR)) {
		set_crosstalk_cancellation_enable(mod, cache_params->ctc_enable);
		flag_process(adapter_data, DAX_PROFILE_MASK, DAX_FLAG_SET);
	}
	if (flag_process(adapter_data, DAX_PROFILE_MASK, DAX_FLAG_READ_AND_CLEAR)) {
		set_profile(mod, cache_params->profile);
		if (!cache_params->content_processing_enable)
			flag_process(adapter_data, DAX_CP_MASK, DAX_FLAG_SET);
	}
	if (flag_process(adapter_data, DAX_CP_MASK, DAX_FLAG_READ_AND_CLEAR))
		set_content_processing_enable(mod,
					      cache_params->content_processing_enable);
	if (flag_process(adapter_data, DAX_VOLUME_MASK, DAX_FLAG_READ_AND_CLEAR))
		set_volume(mod, cache_params->volume);
}

/**
 * @brief Reset the crossfade state machine to the passthrough state.
 *
 * @param[in,out] xf Crossfade context.
 */
static void dax_xfade_reset(struct dax_crossfade *xf)
{
	xf->state = DAX_STATE_BYPASS;
	xf->fade_from_process = false;
	xf->fade_to_process = false;
	xf->gain = 0;
	xf->gain_step = 0;
	xf->fade_left = 0;
	xf->blocks_fed = 0;
}

/**
 * @brief Check whether the DAX algorithm has to be run in the current state.
 *
 * @param[in] xf Crossfade context.
 *
 * @return true when a DAX block has to be submitted in this state.
 */
static bool dax_state_uses_process(const struct dax_crossfade *xf)
{
	switch (xf->state) {
	case DAX_STATE_PREHEAT:
	case DAX_STATE_PROCESS:
		return true;
	case DAX_STATE_FADE_OUT:
		/* The algorithm is still needed when it is the path being faded
		 * out, or when its output is being primed for the fade-in.
		 */
		return xf->fade_from_process || xf->fade_to_process;
	case DAX_STATE_FADE_IN:
		return xf->fade_to_process;
	case DAX_STATE_BYPASS:
	default:
		return false;
	}
}

/**
 * @brief Start priming the algorithm before its output is used.
 *
 * @param[in] mod Pointer to the processing module.
 */
static void dax_start_preheat(struct processing_module *mod)
{
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);
	struct sof_dax *dax_ctx = &adapter_data->dax_ctx;
	struct dax_crossfade *xf = &adapter_data->xfade;

	if (set_enable(mod, 1) != 0) {
		comp_err(mod->dev, "failed to enable DAX, staying in passthrough");
		return;
	}

	flag_process(adapter_data, DAX_DEVICE_MASK, DAX_FLAG_SET);
	flag_process(adapter_data, DAX_VOLUME_MASK, DAX_FLAG_SET);
	dax_apply_configs(mod);

	dax_buffer_discard(&dax_ctx->output_buffer);
	dax_buffer_discard(&xf->passthrough_buffer);
	xf->blocks_fed = 0;
	xf->state = DAX_STATE_PREHEAT;

	comp_info(mod->dev, "switching to processing, preheating %d block(s)",
		  DAX_LATENCY_BLOCKS);
}

/**
 * @brief Start fading out the currently active path.
 *
 * @param[in,out] xf Crossfade context.
 * @param[in] from_process Whether the algorithm path is currently active.
 * @param[in] to_process Whether the algorithm is the destination path.
 */
static void dax_start_fade_out(struct dax_crossfade *xf, bool from_process,
			       bool to_process)
{
	xf->fade_from_process = from_process;
	xf->fade_to_process = to_process;
	xf->state = DAX_STATE_FADE_OUT;
	xf->gain = DAX_GAIN_FULL;
	xf->fade_left = xf->fade_frames;
	xf->gain_step = -(DAX_GAIN_FULL / (int32_t)xf->fade_frames);
}

/**
 * @brief Start fading in the destination path.
 *
 * @param[in,out] xf Crossfade context.
 * @param[in] to_process Whether the algorithm is the destination path.
 */
static void dax_start_fade_in(struct dax_crossfade *xf, bool to_process)
{
	xf->fade_to_process = to_process;
	xf->state = DAX_STATE_FADE_IN;
	xf->gain = 0;
	xf->fade_left = xf->fade_frames;
	xf->gain_step = DAX_GAIN_FULL / (int32_t)xf->fade_frames;
}

static int sof_dax_reset(struct processing_module *mod)
{
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);
	struct sof_dax *dax_ctx;

	/* dax instance will be established on prepare(), and destroyed on reset() */
	if (adapter_data) {
		dax_ctx = &adapter_data->dax_ctx;
		if (flag_process(adapter_data, DAX_PROCESSING_MASK, DAX_FLAG_READ)) {
			flag_process(adapter_data, DAX_RESET_MASK, DAX_FLAG_SET);
		} else {
			dax_unregister_user(mod);
			dax_buffer_release(mod, &dax_ctx->input_buffer);
			dax_buffer_release(mod, &dax_ctx->output_buffer);
			dax_buffer_release(mod, &adapter_data->xfade.passthrough_buffer);
			dax_xfade_reset(&adapter_data->xfade);
		}
	}

	return 0;
}

static int sof_dax_free(struct processing_module *mod)
{
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);
	struct sof_dax *dax_ctx;

	if (adapter_data) {
		dax_ctx = &adapter_data->dax_ctx;
		if (flag_process(adapter_data, DAX_PROCESSING_MASK, DAX_FLAG_READ)) {
			flag_process(adapter_data, DAX_FREE_MASK, DAX_FLAG_SET);
		} else {
			sof_dax_reset(mod);
			dax_buffer_release(mod, &dax_ctx->tuning_file_buffer);
			mod_data_blob_handler_free(mod, dax_ctx->blob_handler);
			dax_ctx->blob_handler = NULL;
			dax_buffer_release(mod, &adapter_data->cache_params.tuning_file);
			mod_free(mod, adapter_data);
			module_set_private_data(mod, NULL);
		}
	}
	return 0;
}

static void check_and_update_state(struct processing_module *mod)
{
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);

	if (!adapter_data)
		return;

	if (flag_process(adapter_data, DAX_FREE_MASK, DAX_FLAG_READ_AND_CLEAR))
		sof_dax_free(mod);
	else if (flag_process(adapter_data, DAX_RESET_MASK, DAX_FLAG_READ_AND_CLEAR))
		sof_dax_reset(mod);
}

static int sof_dax_init(struct processing_module *mod)
{
	struct comp_dev *dev = mod->dev;
	struct module_data *md = &mod->priv;
	struct dax_adapter_data *adapter_data;
	struct dax_cache_params *cache_params;
	struct sof_dax *dax_ctx;

	md->private = mod_zalloc(mod, sizeof(struct dax_adapter_data));
	if (!md->private) {
		comp_err(dev, "failed to allocate %u bytes for initialization",
			 sizeof(struct dax_adapter_data));
		return -ENOMEM;
	}

	adapter_data = module_get_private_data(mod);
	adapter_data->comp_id = dev->ipc_config.id;
	adapter_data->priority = DAX_USER_PRIORITY_DEFAULT;
	k_spinlock_init(&adapter_data->lock);
	dax_ctx = &adapter_data->dax_ctx;
	cache_params = &adapter_data->cache_params;

	cache_params->enable = 0;
	cache_params->profile = 0;
	cache_params->out_device = 0;
	cache_params->ctc_enable = 1;
	cache_params->content_processing_enable = 1;
	cache_params->volume = 1 << 23;

	dax_ctx->enable = cache_params->enable;
	dax_ctx->profile = cache_params->profile;
	dax_ctx->out_device = cache_params->out_device;
	dax_ctx->ctc_enable = cache_params->ctc_enable;
	dax_ctx->content_processing_enable = cache_params->content_processing_enable;
	dax_ctx->volume = cache_params->volume;
	dax_ctx->update_flags = 0;

	dax_ctx->blob_handler = mod_data_blob_handler_new(mod);
	if (!dax_ctx->blob_handler) {
		comp_err(dev, "create blob handler failed");
		mod_free(mod, adapter_data);
		module_set_private_data(mod, NULL);
		return -ENOMEM;
	}

	return 0;
}

static int check_media_format(struct processing_module *mod)
{
	int ret = 0;
	struct comp_dev *dev = mod->dev;
	struct comp_buffer *source = comp_dev_get_first_data_producer(dev);
	struct comp_buffer *sink = comp_dev_get_first_data_consumer(dev);
	const struct audio_stream *src_stream = &source->stream;
	const struct audio_stream *sink_stream = &sink->stream;
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);
	struct sof_dax *dax_ctx = &adapter_data->dax_ctx;

	if (audio_stream_get_frm_fmt(src_stream) != audio_stream_get_frm_fmt(sink_stream) ||
	    sof_to_dax_frame_fmt(audio_stream_get_frm_fmt(src_stream)) == DAX_FMT_UNSUPPORTED) {
		comp_err(dev, "unsupported format, source %d, sink %d",
			 audio_stream_get_frm_fmt(src_stream),
			 audio_stream_get_frm_fmt(sink_stream));
		ret = -EINVAL;
	}

	if (audio_stream_get_rate(src_stream) != audio_stream_get_rate(sink_stream) ||
	    sof_to_dax_sample_rate(audio_stream_get_rate(src_stream)) == DAX_RATE_UNSUPPORTED) {
		comp_err(dev, "unsupported sample rate, source %d, sink %d",
			 audio_stream_get_rate(src_stream), audio_stream_get_rate(sink_stream));
		ret = -EINVAL;
	}

	if (audio_stream_get_channels(sink_stream) != 2 ||
	    sof_to_dax_channels(audio_stream_get_channels(src_stream)) ==
				DAX_CHANNLES_UNSUPPORTED) {
		comp_err(dev, "unsupported number of channels, source %d, sink %d",
			 audio_stream_get_channels(src_stream),
			 audio_stream_get_channels(sink_stream));
		ret = -EINVAL;
	}

	if (audio_stream_get_buffer_fmt(src_stream) != audio_stream_get_buffer_fmt(sink_stream) ||
	    sof_to_dax_buffer_layout(audio_stream_get_buffer_fmt(src_stream)) ==
				     DAX_BUFFER_LAYOUT_UNSUPPORTED) {
		comp_err(dev, "unsupported buffer layout %d",
			 audio_stream_get_buffer_fmt(src_stream));
		ret = -EINVAL;
	}

	if (ret != 0)
		return ret;

	dax_ctx->input_media_format.data_format =
		sof_to_dax_frame_fmt(audio_stream_get_frm_fmt(src_stream));
	dax_ctx->input_media_format.sampling_rate =
		sof_to_dax_sample_rate(audio_stream_get_rate(src_stream));
	dax_ctx->input_media_format.num_channels =
		sof_to_dax_channels(audio_stream_get_channels(src_stream));
	dax_ctx->input_media_format.layout =
		sof_to_dax_buffer_layout(audio_stream_get_buffer_fmt(src_stream));
	dax_ctx->input_media_format.bytes_per_sample = audio_stream_sample_bytes(src_stream);

	dax_ctx->output_media_format.data_format =
		sof_to_dax_frame_fmt(audio_stream_get_frm_fmt(sink_stream));
	dax_ctx->output_media_format.sampling_rate =
		sof_to_dax_sample_rate(audio_stream_get_rate(sink_stream));
	dax_ctx->output_media_format.num_channels =
		sof_to_dax_channels(audio_stream_get_channels(sink_stream));
	dax_ctx->output_media_format.layout =
		sof_to_dax_buffer_layout(audio_stream_get_buffer_fmt(sink_stream));
	dax_ctx->output_media_format.bytes_per_sample = audio_stream_sample_bytes(sink_stream);

	comp_info(dev, "format %d, sample rate %d, channels %d, data format %d",
		  dax_ctx->input_media_format.data_format,
		  dax_ctx->input_media_format.sampling_rate,
		  dax_ctx->input_media_format.num_channels,
		  dax_ctx->input_media_format.data_format);
	return 0;
}

static int sof_dax_prepare(struct processing_module *mod, struct sof_source **sources,
			   int num_of_sources, struct sof_sink **sinks, int num_of_sinks)
{
	int ret;
	struct comp_dev *dev = mod->dev;
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);
	struct sof_dax *dax_ctx = &adapter_data->dax_ctx;
	struct dax_crossfade *xf = &adapter_data->xfade;
	uint32_t ibs, obs, pt_bytes, xfade_bytes;
	uint32_t period_frames;

	if (num_of_sources != 1 || num_of_sinks != 1) {
		comp_err(dev, "unsupported number of buffers, in %d, out %d",
			 num_of_sources, num_of_sinks);
		return -EINVAL;
	}

	ret = check_media_format(mod);
	if (ret != 0)
		return ret;

	period_frames = dax_query_period_frames(dax_ctx);
	dax_ctx->sof_period_bytes = dev->frames *
		dax_ctx->output_media_format.num_channels *
		dax_ctx->output_media_format.bytes_per_sample;
	dax_ctx->period_bytes = period_frames *
		dax_ctx->output_media_format.num_channels *
		dax_ctx->output_media_format.bytes_per_sample;
	dax_ctx->period_us = 1000000 * dax_ctx->period_bytes /
		(dax_ctx->output_media_format.bytes_per_sample *
		 dax_ctx->output_media_format.num_channels *
		 dax_ctx->output_media_format.sampling_rate);

	xf->input_period_bytes = period_frames *
		dax_ctx->input_media_format.num_channels *
		dax_ctx->input_media_format.bytes_per_sample;
	xf->fade_frames = MAX((uint32_t)DAX_XFADE_FRAMES, 1U);
	xfade_bytes = xf->fade_frames *
		dax_ctx->output_media_format.num_channels *
		dax_ctx->output_media_format.bytes_per_sample;
	dax_xfade_reset(xf);

	ibs = (period_frames + dev->frames) *
		dax_ctx->input_media_format.num_channels *
		dax_ctx->input_media_format.bytes_per_sample;
	/* The algorithm output is buffered during the preheat and the fade-out
	 * phases, keep room for the latency plus a full fade ramp.
	 */
	obs = xfade_bytes + (DAX_LATENCY_BLOCKS + 2) * dax_ctx->period_bytes +
	      dax_ctx->sof_period_bytes;
	pt_bytes = xfade_bytes + dax_ctx->period_bytes + dax_ctx->sof_period_bytes;

	if (dax_buffer_alloc(mod, &dax_ctx->input_buffer, ibs) != 0) {
		comp_err(dev, "allocate %u bytes failed for input", ibs);
		ret = -ENOMEM;
		goto err;
	}
	if (dax_buffer_alloc(mod, &dax_ctx->output_buffer, obs) != 0) {
		comp_err(dev, "allocate %u bytes failed for output", obs);
		ret = -ENOMEM;
		goto err;
	}
	if (dax_buffer_alloc(mod, &xf->passthrough_buffer, pt_bytes) != 0) {
		comp_err(dev, "allocate %u bytes failed for passthrough", pt_bytes);
		ret = -ENOMEM;
		goto err;
	}
	comp_info(dev, "allocated: ibs %u, obs %u, pt %u, fade %u frames",
		  ibs, obs, pt_bytes, xf->fade_frames);

	dax_register_user(mod);
	dax_check_and_update_instance(mod);

	return 0;

err:
	dax_buffer_release(mod, &dax_ctx->input_buffer);
	dax_buffer_release(mod, &dax_ctx->output_buffer);
	dax_buffer_release(mod, &xf->passthrough_buffer);
	return ret;
}

static int sof_dax_process(struct processing_module *mod, struct sof_source **sources,
			   int num_of_sources, struct sof_sink **sinks, int num_of_sinks)
{
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);
	struct sof_source *source = sources[0];
	struct sof_sink *sink = sinks[0];
	struct sof_dax *dax_ctx;
	struct dax_crossfade *xf;
	enum sof_ipc_frame out_fmt, pt_fmt;
	uint32_t out_frame_bytes, out_channels, pt_frame_bytes, pt_channels;
	bool desired_process;
	int ret;

	if (!adapter_data) {
		comp_err(mod->dev, "invalid adapter data");
		return -EINVAL;
	}

	dax_ctx = &adapter_data->dax_ctx;
	xf = &adapter_data->xfade;
	if (!xf->fade_frames)
		xf->fade_frames = 1;
	if (!xf->input_period_bytes)
		xf->input_period_bytes = dax_ctx->period_bytes;

	flag_process(adapter_data, DAX_PROCESSING_MASK, DAX_FLAG_SET);

	out_fmt = sink_get_frm_fmt(sink);
	out_channels = sink_get_channels(sink);
	out_frame_bytes = sink_get_frame_bytes(sink);
	pt_fmt = source_get_frm_fmt(source);
	pt_channels = source_get_channels(source);
	pt_frame_bytes = source_get_frame_bytes(source);

	/* Keep the DAX instance ownership up to date with the user priorities */
	dax_check_and_update_instance(mod);
	desired_process = adapter_data->cache_params.enable && dax_ctx->p_dax;

	/* The instance may be destroyed at any time when a higher priority user
	 * needs the memory. In that case the algorithm output is not available
	 * anymore, only a fade-in of the passthrough path can be performed.
	 */
	if (!dax_ctx->p_dax && (xf->state == DAX_STATE_PROCESS ||
				(xf->state == DAX_STATE_FADE_OUT &&
				 (xf->fade_from_process || xf->fade_to_process)) ||
				(xf->state == DAX_STATE_FADE_IN && xf->fade_to_process))) {
		set_enable(mod, 0);
		dax_buffer_discard(&dax_ctx->output_buffer);
		dax_start_fade_in(xf, false);
	}

	if (dax_state_uses_process(xf) && dax_ctx->enable)
		dax_apply_configs(mod);

	switch (xf->state) {
	case DAX_STATE_BYPASS:
		/* The input buffer is used as a passthrough FIFO here */
		dax_source_to_buffers(source, &dax_ctx->input_buffer, NULL,
				      dax_ctx->input_buffer.free);
		dax_buffer_to_sink(sink, &dax_ctx->input_buffer, pt_frame_bytes,
				   pt_channels, pt_fmt, NULL);
		/* Start preheating once there is no passthrough data pending */
		if (desired_process && !dax_ctx->input_buffer.avail)
			dax_start_preheat(mod);
		break;

	case DAX_STATE_PREHEAT:
		/* Output passthrough while the algorithm is being primed */
		dax_source_to_buffers(source, &dax_ctx->input_buffer,
				      &xf->passthrough_buffer,
				      MIN(dax_ctx->input_buffer.free,
					  xf->passthrough_buffer.free));
		dax_buffer_to_sink(sink, &xf->passthrough_buffer, pt_frame_bytes,
				   pt_channels, pt_fmt, NULL);

		ret = dax_run_process(mod);
		if (ret > 0) {
			xf->blocks_fed++;
			/* Output of the first blocks is still the latency history */
			if (xf->blocks_fed <= DAX_LATENCY_BLOCKS)
				dax_buffer_discard(&dax_ctx->output_buffer);
		}

		if (!desired_process) {
			/* Requested back to passthrough before the algorithm was
			 * primed: fade the passthrough out and bypass back in.
			 */
			dax_start_fade_out(xf, false, false);
		} else if (xf->blocks_fed > DAX_LATENCY_BLOCKS &&
			   dax_ctx->output_buffer.avail) {
			dax_start_fade_out(xf, false, true);
		}
		break;

	case DAX_STATE_FADE_OUT:
		if (xf->fade_from_process) {
			/* Fade the algorithm output out, keep it fed */
			dax_source_to_buffers(source, &dax_ctx->input_buffer, NULL,
					      dax_ctx->input_buffer.free);
			dax_run_process(mod);
			dax_buffer_to_sink(sink, &dax_ctx->output_buffer,
					   out_frame_bytes, out_channels, out_fmt, xf);
		} else if (xf->fade_to_process) {
			/* Fade the passthrough out and prime algorithm output */
			dax_source_to_buffers(source, &dax_ctx->input_buffer,
					      &xf->passthrough_buffer,
					      MIN(dax_ctx->input_buffer.free,
						  xf->passthrough_buffer.free));
			dax_run_process(mod);
			dax_buffer_to_sink(sink, &xf->passthrough_buffer,
					   pt_frame_bytes, pt_channels, pt_fmt, xf);
		} else {
			/* Preheat aborted, only the passthrough is faded out */
			dax_source_to_buffers(source, &xf->passthrough_buffer, NULL,
					      xf->passthrough_buffer.free);
			dax_buffer_to_sink(sink, &xf->passthrough_buffer,
					   pt_frame_bytes, pt_channels, pt_fmt, xf);
		}

		if (!xf->fade_left) {
			/* Fade-out done: the old path is at silence now */
			if (!xf->fade_to_process) {
				/* The destination is the passthrough path, the
				 * algorithm is not needed anymore.
				 */
				set_enable(mod, 0);
				dax_buffer_discard(&dax_ctx->output_buffer);
				dax_buffer_discard(&xf->passthrough_buffer);
			}

			comp_info(mod->dev, "faded out %s, fading in %s",
				  xf->fade_from_process ? "processing" : "passthrough",
				  xf->fade_to_process ? "processing" : "passthrough");
			dax_start_fade_in(xf, xf->fade_to_process);
		}
		break;

	case DAX_STATE_FADE_IN:
		dax_source_to_buffers(source, &dax_ctx->input_buffer, NULL,
				      dax_ctx->input_buffer.free);
		if (xf->fade_to_process) {
			dax_run_process(mod);
			dax_buffer_to_sink(sink, &dax_ctx->output_buffer,
					   out_frame_bytes, out_channels, out_fmt, xf);
		} else {
			/* The input buffer holds the passthrough stream */
			dax_buffer_to_sink(sink, &dax_ctx->input_buffer,
					   pt_frame_bytes, pt_channels, pt_fmt, xf);
		}

		if (!xf->fade_left) {
			dax_buffer_discard(&xf->passthrough_buffer);
			xf->state = xf->fade_to_process ? DAX_STATE_PROCESS :
							  DAX_STATE_BYPASS;
			comp_info(mod->dev, "switched to %s path", xf->fade_to_process ?
				  "processing" : "passthrough");
		}
		break;

	case DAX_STATE_PROCESS:
		dax_source_to_buffers(source, &dax_ctx->input_buffer, NULL,
				      dax_ctx->input_buffer.free);
		dax_run_process(mod);
		dax_buffer_to_sink(sink, &dax_ctx->output_buffer, out_frame_bytes,
				   out_channels, out_fmt, NULL);
		if (!desired_process)
			dax_start_fade_out(xf, true, false);
		break;

	default:
		break;
	}

	flag_process(adapter_data, DAX_PROCESSING_MASK, DAX_FLAG_CLEAR);
	check_and_update_state(mod);

	return 0;
}

static int sof_dax_set_configuration(struct processing_module *mod, uint32_t config_id,
				     enum module_cfg_fragment_position pos,
				     uint32_t data_offset_size, const uint8_t *fragment,
				     size_t fragment_size,
				     uint8_t *response, size_t response_size)
{
	int ret;
	struct comp_dev *dev = mod->dev;
	struct dax_adapter_data *adapter_data = module_get_private_data(mod);
	struct sof_dax *dax_ctx = &adapter_data->dax_ctx;
	int32_t dax_param_id = 0;
	int32_t val;
	void *data = &val;
	size_t data_size = sizeof(val);

#if CONFIG_IPC_MAJOR_4
	const struct sof_ipc4_control_msg_payload *ctl = NULL;

	if (fragment_size == 0)
		return 0;

	switch (config_id) {
	case 0: /* IPC4_VOLUME */
		/* ipc4_peak_volume_config::target_volume */
		val = ((const int32_t *)fragment)[1];
		val = sat_int32(Q_SHIFT_RND((int64_t)val, 31, 23));
		dax_param_id = DAX_PARAM_ID_ABSOLUTE_VOLUME;
		break;
	case SOF_IPC4_SWITCH_CONTROL_PARAM_ID:
		ctl = (const struct sof_ipc4_control_msg_payload *)fragment;
		if (ctl->num_elems != 1)
			return -EINVAL;

		val = ctl->chanv[0].value;
		switch (ctl->id) {
		case DAX_SWITCH_ENABLE_CONTROL_ID:
			dax_param_id = DAX_PARAM_ID_ENABLE;
			break;
		case DAX_SWITCH_CP_CONTROL_ID:
			dax_param_id = DAX_PARAM_ID_CP_ENABLE;
			break;
		case DAX_SWITCH_CTC_CONTROL_ID:
			dax_param_id = DAX_PARAM_ID_CTC_ENABLE;
			break;
		default:
			comp_err(dev, "unknown switch control %d", ctl->id);
			return -EINVAL;
		}
		break;
	case SOF_IPC4_ENUM_CONTROL_PARAM_ID:
		ctl = (const struct sof_ipc4_control_msg_payload *)fragment;
		if (ctl->num_elems != 1)
			return -EINVAL;

		val = ctl->chanv[0].value;
		switch (ctl->id) {
		case DAX_ENUM_PROFILE_CONTROL_ID:
			dax_param_id = DAX_PARAM_ID_PROFILE;
			break;
		case DAX_ENUM_DEVICE_CONTROL_ID:
			dax_param_id = DAX_PARAM_ID_OUT_DEVICE;
			break;
		default:
			comp_err(dev, "unknown enum control %d", ctl->id);
			return -EINVAL;
		}
		break;
	case IPC4_DAX_MODULE_CFG_PARAM_BLOB:
		dax_param_id = DAX_PARAM_ID_TUNING_FILE;
		break;
	default:
		comp_err(dev, "unsupported config id %u", config_id);
		return -EINVAL;
	}
#else
	struct sof_ipc_ctrl_data *ctl = (struct sof_ipc_ctrl_data *)fragment;

	/*
	 * IPC3 SET_VALUE (volume/switch/enum) always arrives with fragment_size 0,
	 * only SET_DATA (binary) carries a payload size.
	 */
	switch (ctl->cmd) {
	case SOF_CTRL_CMD_VOLUME:
		if (ctl->num_elems < 1)
			return -EINVAL;

		/* Host linear volume is Q8.16, DAX expects Q8.23 */
		val = sat_int32(Q_SHIFT_LEFT((int64_t)ctl->chanv[0].value, 16, 23));
		dax_param_id = DAX_PARAM_ID_ABSOLUTE_VOLUME;
		break;
	case SOF_CTRL_CMD_SWITCH:
		if (ctl->num_elems != 1)
			return -EINVAL;

		val = ctl->chanv[0].value;
		switch (ctl->index) {
		case DAX_SWITCH_ENABLE_CONTROL_ID:
			dax_param_id = DAX_PARAM_ID_ENABLE;
			break;
		case DAX_SWITCH_CP_CONTROL_ID:
			dax_param_id = DAX_PARAM_ID_CP_ENABLE;
			break;
		case DAX_SWITCH_CTC_CONTROL_ID:
			dax_param_id = DAX_PARAM_ID_CTC_ENABLE;
			break;
		default:
			comp_err(dev, "unknown switch control %d", ctl->index);
			return -EINVAL;
		}
		break;
	case SOF_CTRL_CMD_ENUM:
		if (ctl->num_elems != 1)
			return -EINVAL;

		val = ctl->chanv[0].value;
		switch (ctl->index) {
		case DAX_ENUM_PROFILE_CONTROL_ID:
			dax_param_id = DAX_PARAM_ID_PROFILE;
			break;
		case DAX_ENUM_DEVICE_CONTROL_ID:
			dax_param_id = DAX_PARAM_ID_OUT_DEVICE;
			break;
		default:
			comp_err(dev, "unknown enum control %d", ctl->index);
			return -EINVAL;
		}
		break;
	case SOF_CTRL_CMD_BINARY:
		if (fragment_size == 0)
			return 0;
		dax_param_id = DAX_PARAM_ID_TUNING_FILE;
		break;
	default:
		comp_err(dev, "unsupported control cmd %d", ctl->cmd);
		return -EINVAL;
	}
#endif

	if (dax_param_id == DAX_PARAM_ID_TUNING_FILE) {
		ret = comp_data_blob_set(dax_ctx->blob_handler, pos,
					 data_offset_size, fragment, fragment_size);
		if (ret != 0)
			return ret;

		/* Non-final fragments are only accumulated, nothing to cache yet */
		if (pos != MODULE_CFG_FRAGMENT_LAST && pos != MODULE_CFG_FRAGMENT_SINGLE)
			return 0;

		data = comp_get_data_blob(dax_ctx->blob_handler, &data_size, NULL);
		if (!data || data_size == 0)
			return 0;
	}

	return dax_cache_config(mod, dax_param_id, data, (uint32_t)data_size);
}

static const struct module_interface dolby_dax_audio_processing_interface = {
	.init = sof_dax_init,
	.prepare = sof_dax_prepare,
	.process = sof_dax_process,
	.set_configuration = sof_dax_set_configuration,
	.reset = sof_dax_reset,
	.free = sof_dax_free,
};

#if CONFIG_COMP_DOLBY_DAX_AUDIO_PROCESSING_MODULE
/* modular: llext dynamic link */

#include <module/module/api_ver.h>
#include <module/module/llext.h>
#include <rimage/sof/user/manifest.h>

static const struct sof_man_module_manifest main_manifest __section(".module") __used = {
	.module = {
		.name = "DAX",
		.uuid = SOF_REG_UUID(dolby_dax_audio_processing),
		.entry_point = (uint32_t)(&dolby_dax_audio_processing_interface),
		.instance_max_count = DAX_MAX_INSTANCE,
		.type = {
			.load_type = SOF_MAN_MOD_TYPE_LLEXT,
			.domain_dp = 1,
		},
		.affinity_mask = 7,
	}
};

SOF_LLEXT_BUILDINFO;

#else

/* unused with Zephyr, generates no output */
DECLARE_TR_CTX(dolby_dax_audio_processing_tr, SOF_UUID(dolby_dax_audio_processing_uuid),
	       LOG_LEVEL_INFO);
DECLARE_MODULE_ADAPTER(dolby_dax_audio_processing_interface, dolby_dax_audio_processing_uuid,
		       dolby_dax_audio_processing_tr);
SOF_MODULE_INIT(dolby_dax_audio_processing,
		sys_comp_module_dolby_dax_audio_processing_interface_init);

#endif
