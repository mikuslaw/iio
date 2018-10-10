#include <stdio.h>
#include <iio.h>
#include <thread>
#include <cstring>
#include <cmath>
#include <chrono>
#include <fftw3.h>
#include <stdio.h>

int process(struct iio_buffer* rxbuf, struct iio_buffer* txbuf);
int prepare_sine(int16_t* re, int16_t* im, size_t size);
void print_complex_vector(const fftw_complex* vec, const size_t size);
int calculate_amplitude_and_phase(int16_t* re, int16_t* im, size_t size, int mult, double &real, double &imaginary);

int main (int argc, char **argv)
{
	size_t buffer_size = 32*1024;
	long long loopback = 1;
	long long sampling_rate = 1000000;

  printf("Opening contex.\n");
	struct iio_context* ctx = iio_create_context_from_uri("ip:pluto.local");
  printf("Opened contex.\n");
 
  printf("Opening phy device.\n");
	struct iio_device* phy_device = iio_context_find_device(ctx, "ad9361-phy");
  printf("Opened phy device.\n");

  printf("Opening dds device.\n");
	struct iio_device* dds_device = iio_context_find_device(ctx, "dds");
	printf("Opened dds device.\n");

  printf("Opening lpc devices.\n");
	struct iio_device* lpc_rx_device = iio_context_find_device(ctx, "cf-ad9361-lpc");
	if(lpc_rx_device==NULL) {
		printf("Couldn't open lpc_rx device.\n");
	}
	struct iio_device* lpc_tx_device = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
	if(lpc_tx_device==NULL) {
		printf("Couldn't open lpc_tx device.\n");
	}
  printf("Opened lpc devices.\n");

	// Loopback on digital
	int result = iio_device_debug_attr_write_longlong(phy_device, "loopback", loopback);
	printf("Set loopback value. Result: %d.\n", result);
	
	// Enable dds
	// struct iio_channel* dds_lpc_channel = iio_device_find_channel(lpc_tx_device, "altvoltage0", true);
	// struct iio_channel* dds_channel = iio_device_find_channel(dds_device, "altvoltage0", true);
	// iio_channel_attr_write_longlong(lpc_tx_device, "frequency", 5000);
	// iio_channel_attr_write_longlong(dds_channel, "scale", 0.4);
	// iio_device_debug_attr_write_longlong(dds_device, "loopback", 1);
	
	// TX1
	printf("Opening tx and tx_lo devices.\n");
	struct iio_channel* tx_channel_lo = iio_device_find_channel(phy_device, "altvoltage1", true);
	struct iio_channel* tx_channel = iio_device_find_channel(phy_device, "voltage0", true);
	iio_channel_attr_write_longlong(tx_channel_lo, "frequency", 2400000000); /* TX LO frequency 2.4GHz */
	iio_channel_attr_write_longlong(tx_channel, "sampling_frequency", sampling_rate); /* TX baseband rate 0.5 MSPS */
	//rf_bandwidth
	//rf_port_select
	printf("Opening tx and tx_lo devices.\n");

	// RX1
	printf("Opening rx and rx_lo devices.\n");
	struct iio_channel* rx_channel_lo = iio_device_find_channel(phy_device, "altvoltage0", true);
	struct iio_channel* rx_channel = iio_device_find_channel(phy_device, "voltage0", false);
	iio_channel_attr_write_longlong(rx_channel_lo, "frequency", 2400000000); /* RX LO frequency 2.4GHz */
	iio_channel_attr_write_longlong(rx_channel, "sampling_frequency", sampling_rate); /* RX baseband rate 0.5 MSPS */
	printf("Opened rx and rx_lo devices.\n");

	printf("Opening receive channels.\n");
	struct iio_channel* rx0_q = iio_device_find_channel(lpc_rx_device, "voltage1", false);
	struct iio_channel* rx0_i = iio_device_find_channel(lpc_rx_device, "voltage0", false);
	if(rx0_q == NULL || rx0_i == NULL) {
		printf("Failed to create rx channel.");
		return -1;
	}
	iio_channel_enable(rx0_q);
	iio_channel_enable(rx0_i);
	printf("Opened receive channels.\n");

	printf("Channel is scan element> rx0_q:%d, rx0_i:%d.\n", iio_channel_is_scan_element(rx0_q), iio_channel_is_scan_element(rx0_i));
	printf("Channel is enabled> rx0_q:%d, rx0_i:%d.\n", iio_channel_is_enabled(rx0_q), iio_channel_is_enabled(rx0_i));
	printf("Channel is output> rx0_q:%d, rx0_i:%d.\n", iio_channel_is_output(rx0_q), iio_channel_is_output(rx0_i));

	printf("Opening transmit channels.\n");
	struct iio_channel* tx0_q = iio_device_find_channel(lpc_tx_device, "voltage1", true);
	struct iio_channel* tx0_i = iio_device_find_channel(lpc_tx_device, "voltage0", true);
	if(tx0_q == NULL || tx0_i == NULL) {
		printf("Failed to create tx channel.");
		return -1;
	}
	iio_channel_enable(tx0_q);
	iio_channel_enable(tx0_i);
	printf("Opened transmit channels.\n");

	printf("Channel is scan element> tx0_q:%d, tx0_i:%d.\n", iio_channel_is_scan_element(tx0_q), iio_channel_is_scan_element(tx0_i));
	printf("Channel is enabled> tx0_q:%d, tx0_i:%d.\n", iio_channel_is_enabled(tx0_q), iio_channel_is_enabled(tx0_i));
	printf("Channel is output> tx0_q:%d, tx0_i:%d.\n", iio_channel_is_output(tx0_q), iio_channel_is_output(tx0_i));

	iio_device_set_kernel_buffers_count(lpc_rx_device, 2);
	iio_device_set_kernel_buffers_count(lpc_tx_device, 2);

	printf("Creating receive buffer.\n");
  struct iio_buffer* rxbuf;
	rxbuf = iio_device_create_buffer(lpc_rx_device, buffer_size, false);
	if (!rxbuf) {
		perror("Could not create RX buffer");
		exit(0);
	}
	printf("Created receive buffer.\n");

	printf("Creating transmit buffer.\n");
  struct iio_buffer* txbuf;
	txbuf = iio_device_create_buffer(lpc_tx_device, buffer_size, false);
	if (!txbuf) {
		perror("Could not create TX buffer");
		exit(0);
	}
	printf("Created transmit buffer.\n");

	printf("Preparing tx samples.\n");
	int16_t* tx_real = new int16_t[buffer_size*sizeof(int16_t)];
	int16_t* tx_imag = new int16_t[buffer_size*sizeof(int16_t)];

	// Use FFT to create a sinewave.
	prepare_sine(tx_real, tx_imag, buffer_size);

	// Calculate input sinewave 
	double amplitude, phase;
	calculate_amplitude_and_phase(tx_real, tx_imag, buffer_size, 4, amplitude, phase);
	printf("Prepared tx. [Out] Amplitude: %f, Phase: %f.\n", amplitude, phase);

	printf("Preparing place for rx samples.\n");
	int16_t* rx_real = new int16_t[buffer_size*sizeof(int16_t)];
	int16_t* rx_imag = new int16_t[buffer_size*sizeof(int16_t)];
	printf("Prepared place for rx samples.\n");
 
  printf("Processing start...\n");
	size_t nbytes_tx_q = iio_channel_write(tx0_q, txbuf, tx_real, buffer_size*sizeof(int16_t));
	size_t nbytes_tx_i = iio_channel_write(tx0_i, txbuf, tx_imag, buffer_size*sizeof(int16_t));

	iio_buffer_push(txbuf);
	iio_buffer_push(txbuf);
	iio_buffer_push(txbuf);

	for(int i=0;i<20;i++) {
		iio_buffer_push(txbuf);
		iio_buffer_refill(rxbuf);

		// Grab 10th buffer for display.
		if (i==10) {
			// Service receive.
			size_t nbytes_rx_q = iio_channel_read(rx0_q, rxbuf, rx_real, buffer_size*sizeof(int16_t));
			size_t nbytes_rx_i = iio_channel_read(rx0_i, rxbuf, rx_imag, buffer_size*sizeof(int16_t));
			// printf("Read rx samples. Re: %ld, Im: %ld.\n", nbytes_rx_q, nbytes_rx_i);
		}	
		size_t nbytes_tx_q = iio_channel_write(tx0_q, txbuf, tx_real, buffer_size*sizeof(int16_t));
		size_t nbytes_tx_i = iio_channel_write(tx0_i, txbuf, tx_imag, buffer_size*sizeof(int16_t));
		// printf("Stored tx samples. Re: %ld, Im: %ld.\n", nbytes_tx_q, nbytes_tx_i);
	}
	printf("Processing end.\n");

	for(int i=0;i<buffer_size;i++) {
		rx_real[i] = rx_real[i]<<4;
		rx_imag[i] = rx_imag[i]<<4;
	}

	calculate_amplitude_and_phase(rx_real, rx_imag, buffer_size/4, 1, amplitude, phase);
	printf("Received rx. [In] Amplitude: %f, Phase: %f.\n", amplitude, phase);

	printf("Print both buffers.\n\n");
	printf("TX                    RX\n");

	for(int i=0;i<buffer_size;i++) {
		printf("%8d: %8d  %8d i            %8d  %8d i\n",i, tx_real[i], tx_imag[i], rx_real[i], rx_imag[i]);
	}

	iio_buffer_destroy(rxbuf); 
	iio_buffer_destroy(txbuf); 
	iio_context_destroy(ctx);

	delete[] rx_real;
	delete[] rx_imag;
	delete[] tx_real;
	delete[] tx_imag;
 
	return 0;
} 

int calculate_amplitude_and_phase(int16_t* re, int16_t* im, size_t size, int mult, double &mag, double &phase) {
  fftw_complex *in, *out;
  fftw_plan my_plan;
  in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex)*size);
  out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex)*size);

	for(int i=0;i<size;i++) {
		in[i][0] = static_cast<double>(re[i]);
		in[i][1] = static_cast<double>(im[i]);
	}

	std::memset(out, 0, sizeof(fftw_complex)*size);
	my_plan = fftw_plan_dft_1d(size, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
	fftw_execute(my_plan);

	for(int i=0;i<size;i++) {
		out[i][0] = out[i][0]/size;
		out[i][1] = out[i][1]/size;
	}
	// printf("In:\n");
	// print_complex_vector(in, size);
	// printf("Out:\n");
	// print_complex_vector(out, size);

	mag = sqrt(pow(out[mult][0],2) + pow(out[mult][1],2));
	phase = atan2(out[mult][0],out[mult][1]);

	fftw_destroy_plan(my_plan);
  fftw_free(in);
  fftw_free(out);

	return 0;
}

int prepare_sine(int16_t* re, int16_t* im, size_t size) {
  const int mult = 4;
  const int sin_idx = mult;
  fftw_complex *in, *out;
  fftw_plan my_plan;
  in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex)*size);
  out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex)*size);

	std::memset(in, 0, sizeof(fftw_complex)*size);
	std::memset(out, 0, sizeof(fftw_complex)*size);
  in[sin_idx][0] = 1;
  in[sin_idx][1] = 0;

  // printf("IN:\n");
  // print_complex_vector(in, size);
  // printf("OUT:\n");
  // print_complex_vector(out, size);

  my_plan = fftw_plan_dft_1d(size, in, out, FFTW_BACKWARD, FFTW_ESTIMATE);
  fftw_execute(my_plan);
  // printf("OUT after:\n");
  // print_complex_vector(out, size);
  
	for(int i=0;i<size;i++) {
		re[i] = (int16_t(out[i][0]*(INT16_MAX)) & (~int16_t(0xF)));
		im[i] = (int16_t(out[i][1]*(INT16_MAX)) & (~int16_t(0xF)));
	}

	fftw_destroy_plan(my_plan);
  fftw_free(in);
  fftw_free(out);

  return 0;
}

void print_complex_vector(const fftw_complex* vec, const size_t size) {
  for(int i=0;i<size;i++) {
    printf("%+8f %+8f i",vec[i][0], vec[i][1]);
    if((i+1)%4==0) {
      printf("\n");
    } else {
      printf("    ");
    }
  }
  printf("\n\n");
}
