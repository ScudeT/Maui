#include "MS5837.h"

const float MS5837::Pa = 100.0f;
const float MS5837::bar = 0.001f;
const float MS5837::mbar = 1.0f;

const uint8_t MS5837::MS5837_30BA = 0;
const uint8_t MS5837::MS5837_02BA = 1;
const uint8_t MS5837::MS5837_UNRECOGNISED = 255;

MS5837::MS5837() : i2c_fd(-1), fluidDensity(1029), _model(MS5837_UNRECOGNISED) {}

MS5837::~MS5837() {
    if (i2c_fd >= 0) {
        close(i2c_fd);
    }
}

bool MS5837::init(const std::string& i2c_device) {
    i2c_fd = open(i2c_device.c_str(), O_RDWR);
    if (i2c_fd < 0) {
        std::cerr << "Failed to open I2C device: " << i2c_device << std::endl;
        return false;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, 0x76) < 0) {
        std::cerr << "Failed to set I2C address" << std::endl;
        close(i2c_fd);
        return false;
    }

    if (!writeByte(0x1E)) {
        std::cerr << "Failed to reset sensor" << std::endl;
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    uint8_t buffer[2];
    for (uint8_t i = 0; i < 7; ++i) {
        if (!readBytes(0xA0 + i * 2, buffer, 2)) {
            std::cerr << "Failed to read PROM" << std::endl;
            return false;
        }
        C[i] = (buffer[0] << 8) | buffer[1];
    }

    uint8_t crcRead = C[0] >> 12;
    if (crcRead != crc4(C)) {
        std::cerr << "CRC mismatch" << std::endl;
        return false;
    }

    return true;
}

bool MS5837::writeByte(uint8_t reg) {
    return write(i2c_fd, &reg, 1) == 1;
}

bool MS5837::readBytes(uint8_t reg, uint8_t* buffer, size_t length) {
    if (write(i2c_fd, &reg, 1) != 1) {
        return false;
    }
    return read(i2c_fd, buffer, length) == static_cast<ssize_t>(length);
}

void MS5837::setModel(uint8_t model) {
	_model = model;
}

uint8_t MS5837::getModel() {
	return (_model);
}

void MS5837::setFluidDensity(float density) {
	fluidDensity = density;
}


void MS5837::readSensor() {
    if (i2c_fd < 0) {
        std::cerr << "I2C not initialized. Call init() first." << std::endl;
        return;
    }

    uint8_t buffer[3];

    // Request D1 conversion (pressure)
    if (!writeByte(0x4A)) {  // MS5837_CONVERT_D1_8192
        std::cerr << "Failed to start D1 conversion" << std::endl;
        return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));  // Wait for conversion

    // Read D1 result
    if (!readBytes(0x00, buffer, 3)) {  // MS5837_ADC_READ
        std::cerr << "Failed to read D1" << std::endl;
        return;
    }
    D1_pres = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];

    // Request D2 conversion (temperature)
    if (!writeByte(0x5A)) {  // MS5837_CONVERT_D2_8192
        std::cerr << "Failed to start D2 conversion" << std::endl;
        return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));  // Wait for conversion

    // Read D2 result
    if (!readBytes(0x00, buffer, 3)) {  // MS5837_ADC_READ
        std::cerr << "Failed to read D2" << std::endl;
        return;
    }
    D2_temp = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];

    // Perform calculations
    calculate();
}

void MS5837::calculate() {
	// Given C1-C6 and D1, D2, calculated TEMP and P
	// Do conversion first and then second order temp compensation

	int32_t dT = 0;
	int64_t SENS = 0;
	int64_t OFF = 0;
	int32_t SENSi = 0;
	int32_t OFFi = 0;
	int32_t Ti = 0;
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;

	// Terms called
	dT = D2_temp-uint32_t(C[5])*256l;
	if ( _model == MS5837_02BA ) {
		SENS = int64_t(C[1])*65536l+(int64_t(C[3])*dT)/128l;
		OFF = int64_t(C[2])*131072l+(int64_t(C[4])*dT)/64l;
		P = (D1_pres*SENS/(2097152l)-OFF)/(32768l);
	} else {
		SENS = int64_t(C[1])*32768l+(int64_t(C[3])*dT)/256l;
		OFF = int64_t(C[2])*65536l+(int64_t(C[4])*dT)/128l;
		P = (D1_pres*SENS/(2097152l)-OFF)/(8192l);
	}

	// Temp conversion
	TEMP = 2000l+int64_t(dT)*C[6]/8388608LL;

	//Second order compensation
	if ( _model == MS5837_02BA ) {
		if((TEMP/100)<20){         //Low temp
			Ti = (11*int64_t(dT)*int64_t(dT))/(34359738368LL);
			OFFi = (31*(TEMP-2000)*(TEMP-2000))/8;
			SENSi = (63*(TEMP-2000)*(TEMP-2000))/32;
		}
	} else {
		if((TEMP/100)<20){         //Low temp
			Ti = (3*int64_t(dT)*int64_t(dT))/(8589934592LL);
			OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
			SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
			if((TEMP/100)<-15){    //Very low temp
				OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
				SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
			}
		}
		else if((TEMP/100)>=20){    //High temp
			Ti = 2*(dT*dT)/(137438953472LL);
			OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
			SENSi = 0;
		}
	}

	OFF2 = OFF-OFFi;           //Calculate pressure and temp second order
	SENS2 = SENS-SENSi;

	TEMP = (TEMP-Ti);

	if ( _model == MS5837_02BA ) {
		P = (((D1_pres*SENS2)/2097152l-OFF2)/32768l);
	} else {
		P = (((D1_pres*SENS2)/2097152l-OFF2)/8192l);
	}
}

float MS5837::pressure(float conversion) {
	if ( _model == MS5837_02BA ) {
		return P*conversion/100.0f;
	}
	else {
		return P*conversion/10.0f;
	}
}

float MS5837::temperature() {
	return TEMP/100.0f;
}

// The pressure sensor measures absolute pressure, so it will measure the atmospheric pressure + water pressure
// We subtract the atmospheric pressure to calculate the depth with only the water pressure
// The average atmospheric pressure of 101300 pascal is used for the calcuation, but atmospheric pressure varies
// If the atmospheric pressure is not 101300 at the time of reading, the depth reported will be offset
// In order to calculate the correct depth, the actual atmospheric pressure should be measured once in air, and
// that value should subtracted for subsequent depth calculations.
float MS5837::depth() {
	return (pressure(MS5837::Pa)-101300)/(fluidDensity*9.80665);
}

float MS5837::altitude() {
	return (1-pow((pressure()/1013.25),.190284))*145366.45*.3048;
}


uint8_t MS5837::crc4(uint16_t n_prom[]) {
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0 ; i < 16; i++ ) {
		if ( i%2 == 1 ) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
			if ( n_rem & 0x8000 ) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}
