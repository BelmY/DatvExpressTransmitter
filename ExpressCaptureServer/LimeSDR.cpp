#include "stdafx.h"
#ifdef ENABLE_LIMESDR

#include "LimeSDR.h"
#include <LimeSuite.h>


//#pragma comment(lib, "C:\\Users\\Dell\\LimeSuite\\builddir\\src\\Release\\LimeSuite.lib")



static lms_device_t* device = NULL;
static lms_stream_t streamId;
static int   m_limesdr_status;
BOOL m_limesdr_tx;


int limesdr_write_16_bit_samples(scmplx *s, int len) {

	if(m_limesdr_status == EXP_OK)
		LMS_SendStream(&streamId, s, len, NULL, 1000);

	return 0;
}


int limesdr_init() {

	int res = 0;
	m_limesdr_status = EXP_CONF;

	int n;
	lms_info_str_t list[8];
	if ((n = LMS_GetDeviceList(list)) < 0)
		return -1;

	if (device == NULL) {
		if (LMS_Open(&device, list[0], NULL))
			return -1;
	}


	if (LMS_Init(device) != 0)
		return -1;

	if (LMS_EnableChannel(device, LMS_CH_TX, 0, true) != 0)
		return -1;

	if (LMS_SetAntenna(device, LMS_CH_TX, 0, 1) != 0)
		return -1;

	m_limesdr_status = EXP_OK;

	m_limesdr_tx = FALSE;

	return 0;
}

void limesdr_deinit(void) {
	if (m_limesdr_status != EXP_OK) return;

	LMS_Close(device);
	m_limesdr_status = EXP_CONF;

}

void limesdr_set_freq(double freq) {
	if (m_limesdr_status == EXP_OK)
		LMS_SetLOFrequency(device, LMS_CH_TX, 0, freq);

}

void limesdr_set_level(int level) {

	float_type gain = level / 47.0;
	if (m_limesdr_status == EXP_OK)
		LMS_SetNormalizedGain(device, LMS_CH_TX, 0, gain);
}

int limesdr_set_sr(double sr) {


	if (m_limesdr_status == EXP_OK) {
		float_type freq = 0;;

		if (m_limesdr_tx) {

			LMS_GetLOFrequency(device, LMS_CH_TX, 0, &freq);

			m_limesdr_tx = FALSE;

			LMS_StopStream(&streamId);
			LMS_DestroyStream(device, &streamId);
		
		}		

		LMS_SetSampleRate(device, sr, 2);

		if (freq != 0) {

			limesdr_set_freq(freq);

			limesdr_run();

		}
	}
	return 0;
}

void limesdr_run(void) {

	if (m_limesdr_status == EXP_OK) {

		streamId.channel = 0;
		streamId.fifoSize = 1024 * 128;
		streamId.throughputVsLatency = 1.0;
		streamId.isTx = true;
		streamId.dataFmt = lms_stream_t::LMS_FMT_I16;

		LMS_SetupStream(device, &streamId);

		LMS_StartStream(&streamId);

		m_limesdr_tx = TRUE;
	}
}

#endif