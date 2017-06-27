#include "stdafx.h"
#ifdef ENABLE_LIMESDR

#include "LimeSDR.h"
#include <LimeSuite.h>
#define _USE_MATH_DEFINES // for C++
#include <cmath>


static lms_device_t* device = NULL;
static lms_stream_t streamId;
static int   m_limesdr_status;
BOOL m_limesdr_tx;
double m_sr = 0;
float_type ShiftNCO[16] = { 000000,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
float m_gain = 0.0;

int OVERSAMPLE = 32;


void build_rrc_filter(float_type *filter, float rolloff, float gain, int ntaps, int samples_per_symbol) {
	double a, b, c, d;
	double B = rolloff;// Rolloff factor
	double t = -(ntaps - 1) / 2;// First tap
	double Ts = samples_per_symbol;// Samples per symbol
	double max = 0;
	// Create the filter
	for (int i = 0; i < (ntaps); i++) {
		a = 2.0 * B / (M_PI*sqrt(Ts));
		b = cos((1.0 + B)*M_PI*t / Ts);
		// Check for infinity in calculation (computers don't have infinite precision)
		if (t == 0)
			c = (1.0 - B)*M_PI / (4 * B);
		else
			c = sin((1.0 - B)*M_PI*t / Ts) / (4.0*B*t / Ts);

		d = (1.0 - (4.0*B*t / Ts)*(4.0*B*t / Ts));
		//filter[i] = (b+c)/(a*d);//beardy
		filter[i] = a*(b + c) / d;//nasa
		if (fabs(filter[i]) > max) max = fabs(filter[i]);
		t = t + 1.0;
	}
	// Normalise the filter coefficients
	gain = gain / max;
	for (int i = 0; i < ntaps; i++) {
		
		filter[i] = filter[i] * gain;
	}
}

int limesdr_write_16_bit_samples(scmplx *s, int len) {

	if (m_limesdr_status == EXP_OK)
	{
		//memset(s, 0, len * sizeof(scmplx));
		if(LMS_SendStream(&streamId, s, len, NULL, 1000)!=len) TRACE("!");
	}
	//lms_stream_status_t TxStatus;
	//LMS_GetStreamStatus(&streamId, &TxStatus);

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

	if (LMS_EnableChannel(device, LMS_CH_TX, 0, false) != 0) //Not enable
		return -1;

	if (LMS_SetAntenna(device, LMS_CH_TX, 0, 1) != 0)
		return -1;

	streamId.channel = 0;
	streamId.fifoSize = 500000;
	streamId.throughputVsLatency = 0.0;
	streamId.isTx = true;
	streamId.dataFmt = lms_stream_t::LMS_FMT_I16;
		
	LMS_SetupStream(device, &streamId);

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
	m_gain = gain;
	if (m_limesdr_status == EXP_OK)
		LMS_SetNormalizedGain(device, LMS_CH_TX, 0, gain);
}

int limesdr_set_sr(double sr) {


	if (m_limesdr_status == EXP_OK) {
		float_type freq = 0;;
		m_sr = sr;
		
		/*if (m_limesdr_tx) {

			LMS_GetLOFrequency(device, LMS_CH_TX, 0, &freq);

			m_limesdr_tx = FALSE;

			LMS_StopStream(&streamId);
			LMS_DestroyStream(device, &streamId);
		
		}*/		
		//LMS_SetSampleRate(device, sr, 2);
		lms_range_t Range;
		LMS_GetSampleRateRange(device, LMS_CH_TX, &Range);
		if ((m_sr < Range.min) || (m_sr > Range.max))
		{
			char sDebug[255];
			sprintf_s(sDebug, "Valid SR=%f-%f by %f step", Range.min, Range.max, Range.step);
			AfxMessageBox(sDebug);
		}
		if (m_sr <= 400000)	OVERSAMPLE = 32;
		if((m_sr>400000)&&(m_sr<=800000)) 	OVERSAMPLE = 16;
		if((m_sr>800000)&&(m_sr<=2000000)) OVERSAMPLE = 4;

		if (LMS_SetSampleRate(device, m_sr, OVERSAMPLE)!=0)
		{
			AfxMessageBox("SR Not Set");
		}

		
		//LMS_SetLPF(device, LMS_CH_TX, 0, true);
		/*if (LMS_SetGFIRLPF(device, LMS_CH_TX, 0, true, sr*2) < 0)
		{
			AfxMessageBox("Failed GFIR");
		}*/
			
		/*LMS_GetLOFrequency(device, LMS_CH_TX, 0, &freq);
		if (freq != 0) {
			
			limesdr_set_freq(freq-ShiftNCO[0]);
			//LMS_SetNCOFrequency(device, LMS_CH_TX, 0, ShiftNCO, 0);
			//LMS_SetNCOIndex(device, LMS_CH_TX, 0, 0, true);
			LMS_Calibrate(device, LMS_CH_TX, 0, sr*2, 0);
			//limesdr_run();

		}*/
	}
	return 0;
}

void limesdr_run(void) {

	if (m_limesdr_status == EXP_OK) {

		LMS_EnableChannel(device, LMS_CH_TX, 0, true);
		LMS_SetNormalizedGain(device, LMS_CH_TX, 0, m_gain);

		
		
#define RRC_OVERSAMPLE 1
#define RRC_TAPS 120
		static float_type rrc_coeffs[RRC_TAPS*RRC_OVERSAMPLE];
		build_rrc_filter(rrc_coeffs, 0.35, 0.4, RRC_TAPS*RRC_OVERSAMPLE, RRC_OVERSAMPLE);


		if (LMS_SetGFIRCoeff(device, LMS_CH_TX, 0, LMS_GFIR3, &rrc_coeffs[0], (RRC_TAPS*RRC_OVERSAMPLE))<0)
			AfxMessageBox("Unable to set coeff GFIR");
		if (LMS_SetGFIR(device, LMS_CH_TX, 0, LMS_GFIR3, true)<0)
			AfxMessageBox("Unable to set GFIR");

		LMS_Calibrate(device, LMS_CH_TX, 0,8000000, 0);
		LMS_StartStream(&streamId);

		

		m_limesdr_tx = TRUE;
	}
}

void limesdr_pause(void) {
	if (m_limesdr_status == EXP_OK) {
		m_limesdr_tx = FALSE;
		LMS_StopStream(&streamId);
		LMS_EnableChannel(device, LMS_CH_TX, 0, false);
		//LMS_SetNormalizedGain(device, LMS_CH_TX, 0, 0);
		//LMS_DestroyStream(device, &streamId);
	}

}


#endif