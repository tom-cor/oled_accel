#include "FIRFilter.h"

//	LP filter with 20Hz cutoff
//static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {-0.0020337f,0.0032530f,0.0111339f,0.0000000f,-0.0411319f,-0.0447901f,0.0810403f,0.2922106f,0.4000000f,0.2922106f,0.0810403f,-0.0447901f,-0.0411319f,0.0000000f,0.0111339f,0.0032530f};

//	LP filter with 10Hz cutoff
//static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {-0.0032906f,-0.0052635f,-0.0068811f,0.0000000f,0.0254209f,0.0724719f,0.1311260f,0.1805961f,0.2000000f,0.1805961f,0.1311260f,0.0724719f,0.0254209f,0.0000000f,-0.0068811f,-0.0052635f};

/*

FIR filter designed with http://t-filter.appspot.com

sampling frequency: 200 Hz

* 0 Hz - 10 Hz  gain = 1  desired ripple = 5 dB  actual ripple = 1.62703667829883 dB

* 20 Hz - 100 Hz  gain = 0  desired attenuation = -40 dB  actual attenuation = -48.04743241726081 dB

*/

static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {
  -0.003707033683301887,
  -0.00784146867015878,
  -0.01123166054428762,
  -0.017093859307432513,
  -0.020449123717417243,
  -0.02295287256071017,
  -0.02076754942722178,
  -0.014512676523445778,
  -0.002041583079431125,
  0.015444567497455099,
  0.03781829178667099,
  0.06252567278102884,
  0.0875139376952865,
  0.10953479912803951,
  0.12611537054786248,
  0.13495176220425864,
  0.13495176220425864,
  0.12611537054786248,
  0.10953479912803951,
  0.0875139376952865,
  0.06252567278102884,
  0.03781829178667099,
  0.015444567497455099,
  -0.002041583079431125,
  -0.014512676523445778,
  -0.02076754942722178,
  -0.02295287256071017,
  -0.020449123717417243,
  -0.017093859307432513,
  -0.01123166054428762,
  -0.00784146867015878,
  -0.003707033683301887
};




void FIRFilter_Init(FIRFilter *fir) {

	/* Clear filter buffer */
	for (uint8_t n = 0; n < FIR_FILTER_LENGTH; n++) {

		fir->buf[n] = 0.0f;

	}

	/* Reset buffer index */
	fir->bufIndex = 0;

	/* Clear filter output */
	fir->out = 0.0f;

}

float FIRFilter_Update(FIRFilter *fir, float inp) {

	/* Store latest sample in buffer */
	fir->buf[fir->bufIndex] = inp;

	/* Increment buffer index and wrap around if necessary */
	fir->bufIndex++;

	if (fir->bufIndex == FIR_FILTER_LENGTH) {

		fir->bufIndex = 0;

	}

	/* Compute new output sample (via convolution) */
	fir->out = 0.0f;

	uint8_t sumIndex = fir->bufIndex;

	for (uint8_t n = 0; n < FIR_FILTER_LENGTH; n++) {

		/* Decrement index and wrap if necessary */
		if (sumIndex > 0) {

			sumIndex--;

		} else {

			sumIndex = FIR_FILTER_LENGTH - 1;

		}

		/* Multiply impulse response with shifted input sample and add to output */
		fir->out += FIR_IMPULSE_RESPONSE[n] * fir->buf[sumIndex];

	}

	/* Return filtered output */
	return fir->out;

}
