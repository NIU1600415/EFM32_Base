#include "apds9960.h"
#include "i2c.h"

gesture_data_type gesture_data_;
int gesture_motion_;

bool APDS9960_Init() {
	uint8_t id;

	/* Initialize I2C */
	BSP_I2C_Init(APDS9960_I2C_ADDR);

	/* Read ID register and check against known values for APDS-9960 */
	if (!I2C_ReadRegister(0x92, &id)) {
		return false;
	} else if (!(id == APDS9960_ID_1 || id == APDS9960_ID_2 || id == APDS9960_ID_3)) {
		return false;
	}

	/* Set ENABLE register to 0 (disable all features) */
	if (!setMode(ALL, OFF)) {
		return false;
	}

	/* Set default values for ambient light and proximity registers */
	if (!I2C_WriteRegister(APDS9960_ATIME, DEFAULT_ATIME)) {
		return false;
	}
	if (!I2C_WriteRegister(APDS9960_WTIME, DEFAULT_WTIME)) {
		return false;
	}
	if (!I2C_WriteRegister(APDS9960_PPULSE, DEFAULT_PROX_PPULSE)) {
		return false;
	}
	if (!I2C_WriteRegister(APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR)) {
		return false;
	}
	if (!I2C_WriteRegister(APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL)) {
		return false;
	}
	if (!I2C_WriteRegister(APDS9960_CONFIG1, DEFAULT_CONFIG1)) {
		return false;
	}
	/*if (!setLEDDrive(DEFAULT_LDRIVE)) {
		return false;
	}
	if (!setProximityGain(DEFAULT_PGAIN)) {
		return false;
	}
	if (!setAmbientLightGain(DEFAULT_AGAIN)) {
		return false;
	}
	if (!setProxIntLowThresh(DEFAULT_PILT)) {
		return false;
	}
	if (!setProxIntHighThresh(DEFAULT_PIHT)) {
		return false;
	}
	if (!setLightIntLowThreshold(DEFAULT_AILT)) {
		return false;
	}
	if (!setLightIntHighThreshold(DEFAULT_AIHT)) {
		return false;
	}*/
	if (!I2C_WriteRegister(APDS9960_PERS, DEFAULT_PERS)) {
		return false;
	}
	if (!I2C_WriteRegister(APDS9960_CONFIG2, DEFAULT_CONFIG2)) {
		return false;
	}
	if (!I2C_WriteRegister(APDS9960_CONFIG3, DEFAULT_CONFIG3)) {
		return false;
	}

	/* Set default values for gesture sense registers */
	/*if (!setGestureEnterThresh(DEFAULT_GPENTH)) {
		return false;
	}
	if (!setGestureExitThresh(DEFAULT_GEXTH)) {
		return false;
	}*/
	if (!I2C_WriteRegister(APDS9960_GCONF1, DEFAULT_GCONF1)) {
		return false;
	}
	/*if (!setGestureGain(DEFAULT_GGAIN)) {
		return false;
	}
	if (!setGestureLEDDrive(DEFAULT_GLDRIVE)) {
		return false;
	}
	if (!setGestureWaitTime(DEFAULT_GWTIME)) {
		return false;
	}*/
	if (!I2C_WriteRegister(APDS9960_GOFFSET_U, DEFAULT_GOFFSET)) {
		return false;
	}
	if (!I2C_WriteRegister(APDS9960_GOFFSET_D, DEFAULT_GOFFSET)) {
		return false;
	}
	if (!I2C_WriteRegister(APDS9960_GOFFSET_L, DEFAULT_GOFFSET)) {
		return false;
	}
	if (!I2C_WriteRegister(APDS9960_GOFFSET_R, DEFAULT_GOFFSET)) {
		return false;
	}
	if (!I2C_WriteRegister(APDS9960_GPULSE, DEFAULT_GPULSE)) {
		return false;
	}
	if (!I2C_WriteRegister(APDS9960_GCONF3, DEFAULT_GCONF3)) {
		return false;
	}
	/*if (!setGestureIntEnable(DEFAULT_GIEN)) {
		return false;
	}*/

	return true;
}

bool APDS9960_enableGestureSensor(bool interrupts) {
	/* Enable gesture mode
	   Set ENABLE to 0 (power off)
	   Set WTIME to 0xFF
	   Set AUX to LED_BOOST_300
	   Enable PON, WEN, PEN, GEN in ENABLE
	*/
	//resetGestureParameters();
	if( !I2C_WriteRegister(APDS9960_WTIME, 0xFF) ) {
		return false;
	}
	if( !I2C_WriteRegister(APDS9960_PPULSE, DEFAULT_GESTURE_PPULSE) ) {
		return false;
	}
	/*if( !setLEDBoost(LED_BOOST_300) ) {
		return false;
	}*/
	if( interrupts ) {
		if( !setGestureIntEnable(1) ) {
			return false;
		}
	} else {
		if( !setGestureIntEnable(0) ) {
			return false;
		}
	}
	if( !setGestureMode(1) ) {
		return false;
	}
	/*if( !enablePower() ){
		return false;
	}*/
	if( !setMode(WAIT, 1) ) {
		return false;
	}
	if( !setMode(PROXIMITY, 1) ) {
		return false;
	}
	if( !setMode(GESTURE, 1) ) {
		return false;
	}

	return true;
}

bool APDS9960_isGestureAvailable() {
    uint8_t val;

    /* Read value from GSTATUS register */
    if( !I2C_ReadRegister(APDS9960_GSTATUS, &val) ) {
        return ERROR;
    }

    /* Shift and mask out GVALID bit */
    val &= APDS9960_GVALID;

    /* Return true/false based on GVALID bit */
    if( val == 1) {
        return true;
    } else {
        return false;
    }
}

bool APDS9960_readGesture() {
    uint8_t fifo_level = 0;
    uint8_t fifo_data[128];
    uint8_t gstatus;
    int bytes_read = 0;
    int motion;
    int i;

    /* Make sure that power and gesture is on and data is valid */
    if( !APDS9960_isGestureAvailable() || !(getMode() & 0b01000001) ) {
        return DIR_NONE;
    }

    /* Keep looping as long as gesture data is valid */
    while(1) {

        /* Wait some time to collect next batch of FIFO data */
    	// TODO
        delay(FIFO_PAUSE_TIME);

        /* Get the contents of the STATUS register. Is data still valid? */
        if( !I2C_ReadRegister(APDS9960_GSTATUS, &gstatus) ) {
            return ERROR;
        }

        /* If we have valid data, read in FIFO */
        if( (gstatus & APDS9960_GVALID) == APDS9960_GVALID ) {

            /* Read the current FIFO level */
            if( !I2C_ReadRegister(APDS9960_GFLVL, &fifo_level) ) {
                return ERROR;
            }

            /* If there's stuff in the FIFO, read it into our data block */
            if( fifo_level > 0) {
            	// TODO
                bytes_read = I2C_ReadRegister(  APDS9960_GFIFO_U,
                                                (uint8_t*)fifo_data,
                                                (fifo_level * 4) );
                if( bytes_read == -1 ) {
                    return ERROR;
                }

                /* If at least 1 set of data, sort the data into U/D/L/R */
                if( bytes_read >= 4 ) {
                    for( i = 0; i < bytes_read; i += 4 ) {
                        gesture_data_.u_data[gesture_data_.index] = \
                                                            fifo_data[i + 0];
                        gesture_data_.d_data[gesture_data_.index] = \
                                                            fifo_data[i + 1];
                        gesture_data_.l_data[gesture_data_.index] = \
                                                            fifo_data[i + 2];
                        gesture_data_.r_data[gesture_data_.index] = \
                                                            fifo_data[i + 3];
                        gesture_data_.index++;
                        gesture_data_.total_gestures++;
                    }

                    /* Reset data */
                    gesture_data_.index = 0;
                    gesture_data_.total_gestures = 0;
                }
            }
        } else {

            /* Determine best guessed gesture and clean up */
            delay(FIFO_PAUSE_TIME);
            // TODO
            decodeGesture();
            motion = gesture_motion_;
            // TODO
            resetGestureParameters();
            return motion;
        }
    }
}

uint8_t getMode() {
	uint8_t enable_value;

	/* Read current ENABLE register */
	if (!I2C_ReadRegister(APDS9960_ENABLE, &enable_value)) {
		return ERROR;
	}

	return enable_value;
}

bool setMode(uint8_t mode, uint8_t enable) {
	uint8_t reg_val;

	/* read curent ENABLE register */
	reg_val = getMode();
	if (reg_val == ERROR) {
		return false;
	}

	/* Change bit(s) in ENABLE register */
	enable = enable & 0x01;
	if(mode <= 6 ) {
		if (enable) {
			reg_val |= (1 << mode);
		} else {
			reg_val &= ~(1 << mode);
		}
	} else if (mode == ALL ) {
		if (enable) {
			reg_val = 0x7F;
		} else {
			reg_val = 0x00;
		}
	}

	/* Write value back to ENABLE register */
	if (!I2C_WriteRegister(APDS9960_ENABLE, reg_val)) {
		return false;
	}

	return true;
}

bool setGestureIntEnable(uint8_t enable) {
	uint8_t val;

	/* Read value from GCONF4 register */
	if( !I2C_ReadRegister(APDS9960_GCONF4, &val) ) {
		return false;
	}

	/* Set bits in register to given value */
	enable &= 0b00000001;
	enable = enable << 1;
	val &= 0b11111101;
	val |= enable;

	/* Write register value back into GCONF4 register */
	if( !I2C_WriteRegister(APDS9960_GCONF4, val) ) {
		return false;
	}

	return true;
}

bool setGestureMode(uint8_t mode)
{
    uint8_t val;

    /* Read value from GCONF4 register */
    if( !I2C_ReadRegister(APDS9960_GCONF4, &val) ) {
        return false;
    }

    /* Set bits in register to given value */
    mode &= 0b00000001;
    val &= 0b11111110;
    val |= mode;

    /* Write register value back into GCONF4 register */
    if( !I2C_WriteRegister(APDS9960_GCONF4, val) ) {
        return false;
    }

    return true;
}
