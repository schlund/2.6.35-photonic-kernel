/* arch/arm/mach-msm/qdsp5_comp/adsp_info.c
 *
 * Copyright (c) 2008 QUALCOMM Incorporated.
 * Copyright (c) 2008 QUALCOMM USA, INC.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "adsp.h"


/* Firmware modules */
typedef enum {
   QDSP_MODULE_KERNEL,
   QDSP_MODULE_AFETASK,
   QDSP_MODULE_AUDPLAY0TASK,
   QDSP_MODULE_AUDPLAY1TASK,
   QDSP_MODULE_AUDPLAY2TASK,
   QDSP_MODULE_AUDPLAY3TASK,
   QDSP_MODULE_AUDPLAY4TASK,
   QDSP_MODULE_AUDPPTASK,
   QDSP_MODULE_MIDI,
   QDSP_MODULE_VIDEOTASK,
   QDSP_MODULE_VIDEO_AAC_VOC,
   QDSP_MODULE_VIDEO_AAC_VOC_TURBO,
   QDSP_MODULE_PCM_DEC,
   QDSP_MODULE_GAUDIO,
   QDSP_MODULE_AUDIO_DEC_MP3,
   QDSP_MODULE_AUDIO_DEC_AAC,
   QDSP_MODULE_AUDIO_DEC_WMA,
   QDSP_MODULE_HOSTPCM,
   QDSP_MODULE_DTMF,
   QDSP_MODULE_AUDRECTASK,
   QDSP_MODULE_AUDPREPROCTASK,
   QDSP_MODULE_SBC_ENC,
   QDSP_MODULE_VOC_CDMA,
   QDSP_MODULE_VOC_CDMA_WB,
   QDSP_MODULE_VOC_UMTS,
   QDSP_MODULE_VOC_UMTS_WB,
   QDSP_MODULE_VOC_PCM,
   QDSP_MODULE_VOCENCTASK,
   QDSP_MODULE_VOCDECTASK,
   QDSP_MODULE_VOICEPROCTASK,
   QDSP_MODULE_VIDEOENCTASK,
   QDSP_MODULE_VFETASK,
   QDSP_MODULE_WAV_ENC,
   QDSP_MODULE_AACLC_ENC,
   QDSP_MODULE_VIDEO_AMR,
   QDSP_MODULE_VIDEO_AMR_TURBO,
   QDSP_MODULE_VOC_AMR,
   QDSP_MODULE_VOC_EVRC,
   QDSP_MODULE_VOC_13K,
   QDSP_MODULE_VOC_FGV,
   QDSP_MODULE_VOC_FR,
   QDSP_MODULE_DIAGTASK,
   QDSP_MODULE_JPEGTASK,
   QDSP_MODULE_QCAMTASK,
   QDSP_MODULE_MODMATHTASK,
   QDSP_MODULE_AUDIO_DEC_AAC_BSAC,
   QDSP_MODULE_WM_LP_MODE,
   QDSP_MODULE_WM_TURBO_MODE,
   QDSP_MODULE_VDEC_LP_MODE,
   QDSP_MODULE_VDEC_LP_MODE_TURBO,
   QDSP_MODULE_MAX,
} qdsp_module_type;

#define QDSP_RTOS_MAX_TASK_ID  30U





/* Table of modules indexed by task ID for the HADRON image */
static const qdsp_module_type qdsp_hadron_task_to_module_table[] =
{
  QDSP_MODULE_KERNEL,
  QDSP_MODULE_AFETASK,
  QDSP_MODULE_VOCDECTASK,
  QDSP_MODULE_VOCENCTASK,
  QDSP_MODULE_VIDEOTASK,
  QDSP_MODULE_VIDEOENCTASK,
  QDSP_MODULE_VOICEPROCTASK,
  QDSP_MODULE_VFETASK,
  QDSP_MODULE_JPEGTASK,
  QDSP_MODULE_AUDPPTASK,
  QDSP_MODULE_AUDPLAY0TASK,
  QDSP_MODULE_AUDPLAY1TASK,
  QDSP_MODULE_AUDPLAY2TASK,
  QDSP_MODULE_AUDPLAY3TASK,
  QDSP_MODULE_AUDPLAY4TASK,
  QDSP_MODULE_MAX,
  QDSP_MODULE_AUDRECTASK,
  QDSP_MODULE_AUDPREPROCTASK,
  QDSP_MODULE_MAX,
  QDSP_MODULE_MAX,
  QDSP_MODULE_MAX,
  QDSP_MODULE_MAX,
  QDSP_MODULE_MAX,
  QDSP_MODULE_MAX,
  QDSP_MODULE_MAX,
  QDSP_MODULE_MAX,
  QDSP_MODULE_MAX,
  QDSP_MODULE_MAX,
  QDSP_MODULE_MAX,
  QDSP_MODULE_MAX,
  QDSP_MODULE_DIAGTASK,
  QDSP_MODULE_MAX
};


static uint32_t qdsp_hadron_queue_offset_table[] =
{
  0x100,               /* QDSP_mpuAfeQueue                  */
  0xfc,                /* QDSP_mpuRmtQueue                  */
  0x130,               /* QDSP_mpuVDecCmdQueue              */
  0x134,               /* QDSP_mpuVDecPktQueue              */
  0x12c,               /* QDSP_mpuVEncCmdQueue              */
  0x108,               /* QDSP_rxMpuDecCmdQueue             */
  0x10c,               /* QDSP_rxMpuDecPktQueue             */
  0x110,               /* QDSP_txMpuEncQueue                */
  0x138,               /* QDSP_uPAudPPCmd1Queue             */
  0x13c,               /* QDSP_uPAudPPCmd2Queue             */
  0x140,               /* QDSP_uPAudPPCmd3Queue             */
  0x150,               /* QDSP_uPAudPlay0BitStreamCtrlQueue */
  0x154,               /* QDSP_uPAudPlay1BitStreamCtrlQueue */
  0x158,               /* QDSP_uPAudPlay2BitStreamCtrlQueue */
  0x15c,               /* QDSP_uPAudPlay3BitStreamCtrlQueue */
  0x160,               /* QDSP_uPAudPlay4BitStreamCtrlQueue */
  0x144,               /* QDSP_uPAudPreProcCmdQueue         */
  0x14c,               /* QDSP_uPAudRecBitStreamQueue       */
  0x148,               /* QDSP_uPAudRecCmdQueue             */
  0x11c,               /* QDSP_uPDiagQueue                  */
  0x118,               /* QDSP_uPJpegFTMActionCmdQueue      */
  0x114,               /* QDSP_uPJpegFTMCfgCmdQueue         */
  0x168,               /* QDSP_uPJpegActionCmdQueue         */
  0x164,               /* QDSP_uPJpegCfgCmdQueue            */
  0x104,               /* QDSP_uPVocProcQueue               */
  0x120,               /* QDSP_vfeCommandQueue              */
  0x128,               /* QDSP_vfeCommandScaleQueue         */
  0x124,               /* QDSP_vfeCommandTableQueue         */
  0xf0,                /* QDSP_vfeFTMCommandQueue           */
  0xf8,                /* QDSP_vfeFTMCommandScaleQueue      */
  0xf4                 /* QDSP_vfeFTMCommandTableQueue      */
};

/* Tables to convert tasks to modules */
static qdsp_module_type *qdsp_task_to_module[] = 
{
	qdsp_hadron_task_to_module_table,
};

/* Tables to retrieve queue offsets */
static uint32_t *qdsp_queue_offset_table[] = 
{
	qdsp_hadron_queue_offset_table,
};

#define QDSP_MODULE(n, clkname, clkrate, verify_cmd_func, patch_event_func) \
	{ .name = #n, .pdev_name = "adsp_" #n, .id = QDSP_MODULE_##n, \
	  .clk_name = clkname, .clk_rate = clkrate, \
	  .verify_cmd = verify_cmd_func, .patch_event = patch_event_func }

static struct adsp_module_info module_info[] = {
	QDSP_MODULE(AUDPLAY0TASK, NULL, 0, NULL, NULL),
	QDSP_MODULE(AUDPPTASK, NULL, 0, NULL, NULL),
	QDSP_MODULE(AUDRECTASK, NULL, 0, NULL, NULL),
	QDSP_MODULE(AUDPREPROCTASK, NULL, 0, NULL, NULL),
	QDSP_MODULE(VFETASK, "vfe_clk", 0,
		adsp_vfe_verify_cmd, adsp_vfe_patch_event),
	QDSP_MODULE(QCAMTASK, NULL, 0, NULL, NULL),
	QDSP_MODULE(JPEGTASK, "vdc_clk", 96000000,
		adsp_jpeg_verify_cmd, adsp_jpeg_patch_event),
	QDSP_MODULE(VIDEOTASK, "vdc_clk",
		96000000, adsp_video_verify_cmd, NULL),
	QDSP_MODULE(VDEC_LP_MODE, NULL, 0, NULL, NULL),
	QDSP_MODULE(VIDEOENCTASK, "vdc_clk", 96000000,
		adsp_videoenc_verify_cmd, NULL),
};

int adsp_init_info(struct adsp_info *info)
{
	info->send_irq =   0x00c00200;
	info->read_ctrl =  0x00400038;
	info->write_ctrl = 0x00400034;

	info->max_msg16_size = 193;
	info->max_msg32_size = 9;

	info->max_task_id = 30;
	info->max_module_id = QDSP_MODULE_MAX - 1;
	info->max_queue_id = QDSP_MAX_NUM_QUEUES;
	info->max_image_id = 2;
	info->queue_offset = qdsp_queue_offset_table;
	info->task_to_module = qdsp_task_to_module;

	info->module_count = ARRAY_SIZE(module_info);
	info->module = module_info;
	return 0;
}
