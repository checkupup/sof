/* SPDX-License-Identifier: BSD-3-Clause
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE
 *
 * Copyright(c) 2025 Dolby Laboratories. All rights reserved.
 *
 * Author: Jun Lai <jun.lai@dolby.com>
 */

#ifndef DAX_H
#define DAX_H

#include <stdbool.h>

#include <rtos/atomic.h>
#include <rtos/spinlock.h>
#include <sof/audio/module_adapter/module/generic.h>
#include <dax_inf.h>

#define DAX_USER_ID_INVALID 0
#define DAX_MAX_INSTANCE 2
#define DAX_PARAMS_STR_BUFFER_SIZE 1536

enum dax_user_priority {
	DAX_USER_PRIORITY_DEFAULT = 0,
	DAX_USER_PRIORITY_P0 = 0,
	DAX_USER_PRIORITY_P1 = 1, /* Highest priority */
};

/**
 * @brief DAX control parameters cached from the configuration thread.
 *
 * sof_dax_set_configuration() and sof_dax_process() may run in different
 * threads. Control parameters are only cached here by the configuration
 * thread and are applied to the DAX instance from the processing thread,
 * which avoids concurrent access to struct sof_dax.
 */
struct dax_cache_params {
	int32_t enable;
	int32_t profile;
	int32_t out_device;
	int32_t ctc_enable;
	int32_t content_processing_enable;
	int32_t volume;
	struct dax_buffer tuning_file;
};

/**
 * @brief Audio path states of the DAX crossfade state machine.
 */
enum dax_path_state {
	DAX_STATE_BYPASS,	/**< Passthrough only, the DAX algorithm is idle */
	DAX_STATE_PREHEAT,	/**< DAX is primed while passthrough is output */
	DAX_STATE_FADE_OUT,	/**< Fading the currently active path out */
	DAX_STATE_FADE_IN,	/**< Fading the destination path in */
	DAX_STATE_PROCESS,	/**< DAX processing only */
};

/**
 * @brief Crossfade state hiding the pop caused by an audio path switch.
 *
 * The DAX algorithm has a latency of several blocks, so its output cannot be
 * mixed with the passthrough signal without a delay line. A path switch is
 * therefore performed in two steps: the currently active path is faded out to
 * silence, then the destination path is faded in. This way the two signals are
 * never mixed and the discontinuity is masked by the fade.
 *
 * Requested path changes are ignored while a fade-out/fade-in sequence is in
 * progress and are only evaluated once the sequence completes.
 */
struct dax_crossfade {
	enum dax_path_state state;	/**< Current state of the state machine */
	bool fade_from_process;		/**< Path active when the fade-out started */
	bool fade_to_process;		/**< Destination path of the transition */
	int32_t gain;			/**< Q31 gain applied to the output */
	int32_t gain_step;		/**< Q31 gain change per audio frame */
	uint32_t fade_frames;		/**< Fade-out / fade-in length in frames */
	uint32_t fade_left;		/**< Frames remaining in the current ramp */
	uint32_t blocks_fed;		/**< DAX blocks fed while preheating */
	uint32_t input_period_bytes;	/**< DAX input block size in bytes */
	struct dax_buffer passthrough_buffer; /**< Passthrough FIFO used while switching */
};

struct dax_adapter_data {
	struct dax_cache_params cache_params;
	atomic_t proc_flags;
	uint32_t comp_id;
	int32_t priority;
	struct k_spinlock lock;
	char params_str[DAX_PARAMS_STR_BUFFER_SIZE];
	struct dax_crossfade xfade;
	struct sof_dax dax_ctx;
};

/**
 * @brief Release memory used by a DAX buffer.
 *
 * @param[in] mod Pointer to the processing module.
 * @param[in,out] dax_buff Pointer to the DAX buffer to release.
 */
void dax_buffer_release(struct processing_module *mod, struct dax_buffer *dax_buff);

/**
 * @brief Allocate memory for a DAX buffer.
 *
 * @param[in] mod Pointer to the processing module.
 * @param[in,out] dax_buff Pointer to the DAX buffer to allocate.
 * @param[in] bytes Number of bytes to allocate.
 *
 * @return 0 on success, negative error code on failure.
 */
int dax_buffer_alloc(struct processing_module *mod, struct dax_buffer *dax_buff, uint32_t bytes);

/**
 * @brief Register the current module as a DAX instance user.
 *
 * @param[in] mod Pointer to the processing module.
 *
 * @return 0 on success, negative error code on failure.
 */
int dax_register_user(struct processing_module *mod);

/**
 * @brief Unregister the current module from DAX instance user management.
 *
 * @param[in] mod Pointer to the processing module.
 *
 * @return 0 on success, negative error code on failure.
 */
int dax_unregister_user(struct processing_module *mod);

/**
 * @brief Reconcile DAX instance allocation based on user priority.
 *
 * @param[in] mod Pointer to the processing module.
 *
 * @return 0 on success, negative error code on failure.
 */
int dax_check_and_update_instance(struct processing_module *mod);

#endif /* DAX_H */
