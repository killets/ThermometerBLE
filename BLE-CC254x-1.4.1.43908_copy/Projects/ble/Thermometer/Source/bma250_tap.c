///*! file <BMA2x2 >
//    brief <Sensor driver for BMA2x2> */
//
//#include "bma250.h"
//
//#include "bma250_tap.h"
///*! user defined code to be added here ... */
//static struct bma2x2_t *p_bma2x2;
///*! Based on Bit resolution v_value_u8 should be modified */
//u8 V_BMA2x2RESOLUTION_U8 = BMA2x2_14_RESOLUTION;
//
//
//
///*!
// *	@brief
// *	This function is used for initialize
// *	bus read and bus write functions
// *	assign the chip id and device address
// *	chip id is read in the register 0x00 bit from 0 to 7
// *
// *	@param bma2x2 : structure pointer
// *
// *	@return results of bus communication function
// *	@retval 0 -> Success
// *	@retval -1 -> Error
// *
// *	@note
// *	While changing the parameter of the bma2x2_t
// *	consider the following point:
// *	Changing the reference value of the parameter
// *	will changes the local copy or local reference
// *	make sure your changes will not
// *	affect the reference value of the parameter
// *	(Better case don't change the reference value of the parameter)
// *
//*/
//s8 bma2x2_init(struct bma2x2_t *bma2x2)
//{
//	/*  Variable used to return value of
//	communication routine*/
//	s8 com_rslt = ERROR;
//	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
//	u8 v_config_data_u8 = C_BMA2x2_ZERO_U8X;
//	/* assign bma2x2 ptr */
//	p_bma2x2 = bma2x2;
//	/* read Chip Id */
//	com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
//	(p_bma2x2->dev_addr,
//	BMA2x2_CHIP_ID__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
//	p_bma2x2->chip_id = v_data_u8;    /* get bit slice */
//	/* read the fifo config register and update
//	the value to the fifo_config*/
//	com_rslt += bma2x2_read_reg(BMA2x2_FIFO_MODE_REG,
//	&v_config_data_u8, C_BMA2x2_ONE_U8X);
//	p_bma2x2->fifo_config = v_config_data_u8;
//	return com_rslt;
//}
//
//
//
///*!
// *	@brief This API is used to set
// *  interrupt enable bits of the sensor in the registers 0x16 and 0x17
// *	@note It reads the flat enable, orient enable,
// *	@note single tap enable, double tap enable
// *	@note slope-x enable, slope-y enable, slope-z enable,
// *	@note fifo watermark enable,
// *	@note fifo full enable, data enable, low-g enable,
// *	@note high-z enable, high-y enable
// *	@note high-z enable
// *
// *
// *
// *  @param v_intr_type_u8: The value of interrupts
// *        v_intr_type_u8   |   result
// *       ----------------- | ------------------
// *              0          | BMA2x2_LOW_G_INTR
// *              1          | BMA2x2_HIGH_G_X_INTR
// *              2          | BMA2x2_HIGH_G_Y_INTR
// *              3          | BMA2x2_HIGH_G_Z_INTR
// *              4          | BMA2x2_DATA_ENABLE
// *              5          | SLOPE_X_INTR
// *              6          | SLOPE_Y_INTR
// *              7          | SLOPE_Z_INTR
// *              8          | SINGLE_TAP_INTR
// *              9          | SINGLE_TAP_INTR
// *              10         | ORIENT_INT
// *              11         | FLAT_INT
// *
// *  @param v_value_u8 : The value of interrupts enable
// *        v_value_u8       |   result
// *       ----------------- | ------------------
// *              0x00       | INTR_DISABLE
// *              0x01       | INTR_ENABLE
// *
// *
// *
// *
// *	@return results of bus communication function
// *	@retval 0 -> Success
// *	@retval -1 -> Error
// *
// *
// */
//s8 bma2x2_set_intr_enable(uint8 v_intr_type_u8,
//u8 v_value_u8)
//{
//		/*  Variable used to return value of
//	communication routine*/
//	int8 com_rslt = ERROR;
//	uint8 v_data_u8 = C_BMA2x2_ZERO_U8X;
//	uint8 v_data2_u8 = C_BMA2x2_ZERO_U8X;
//	if (p_bma2x2 == BMA2x2_NULL) {
//		/* Check the struct p_bma2x2 is empty */
//		return E_BMA2x2_NULL_PTR;
//		} else {
//		com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
//		(p_bma2x2->dev_addr, BMA2x2_INTR_ENABLE1_REG,
//		&v_data_u8, C_BMA2x2_ONE_U8X);
//		com_rslt += p_bma2x2->BMA2x2_BUS_READ_FUNC
//		(p_bma2x2->dev_addr, BMA2x2_INTR_ENABLE2_REG,
//		&v_data2_u8, C_BMA2x2_ONE_U8X);
//		v_value_u8 = v_value_u8 & C_BMA2x2_ONE_U8X;
//		switch (v_intr_type_u8) {
//		case BMA2x2_LOW_G_INTR:
//			/* Low G Interrupt  */
//			v_data2_u8 = BMA2x2_SET_BITSLICE(v_data2_u8,
//			BMA2x2_ENABLE_LOW_G_INTR, v_value_u8);
//		break;
//		case BMA2x2_HIGH_G_X_INTR:
//			/* High G X Interrupt */
//			v_data2_u8 = BMA2x2_SET_BITSLICE(v_data2_u8,
//			BMA2x2_ENABLE_HIGH_G_X_INTR, v_value_u8);
//		break;
//		case BMA2x2_HIGH_G_Y_INTR:
//			/* High G Y Interrupt */
//			v_data2_u8 = BMA2x2_SET_BITSLICE(v_data2_u8,
//			BMA2x2_ENABLE_HIGH_G_Y_INTR, v_value_u8);
//		break;
//		case BMA2x2_HIGH_G_Z_INTR:
//			/* High G Z Interrupt */
//			v_data2_u8 = BMA2x2_SET_BITSLICE(v_data2_u8,
//			BMA2x2_ENABLE_HIGH_G_Z_INTR, v_value_u8);
//		break;
//		case BMA2x2_DATA_ENABLE:
//			/*Data En Interrupt  */
//			v_data2_u8 = BMA2x2_SET_BITSLICE(v_data2_u8,
//			BMA2x2_ENABLE_NEW_DATA_INTR, v_value_u8);
//		break;
//		case BMA2x2_SLOPE_X_INTR:
//			/* Slope X Interrupt */
//			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
//			BMA2x2_ENABLE_SLOPE_X_INTR, v_value_u8);
//		break;
//		case BMA2x2_SLOPE_Y_INTR:
//			/* Slope Y Interrupt */
//			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
//			BMA2x2_ENABLE_SLOPE_Y_INTR, v_value_u8);
//		break;
//		case BMA2x2_SLOPE_Z_INTR:
//			/* Slope Z Interrupt */
//			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
//			BMA2x2_ENABLE_SLOPE_Z_INTR, v_value_u8);
//		break;
//		case BMA2x2_SINGLE_TAP_INTR:
//			/* Single Tap Interrupt */
//			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
//				BMA2x2_ENABLE_SINGLE_TAP_INTR, v_value_u8);
//		break;
//		case BMA2x2_DOUBLE_TAP_INTR:
//			/* Double Tap Interrupt */
//			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
//				BMA2x2_ENABLE_DOUBLE_TAP_INTR, v_value_u8);
//		break;
//		case BMA2x2_ORIENT_INTR:
//			/* Orient Interrupt  */
//			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
//			BMA2x2_ENABLE_ORIENT_INTR, v_value_u8);
//		break;
//		case BMA2x2_FLAT_INTR:
//			/* Flat Interrupt */
//			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
//			BMA2x2_ENABLE_FLAT_INTR, v_value_u8);
//		break;
//		default:
//			com_rslt = E_OUT_OF_RANGE;
//		break;
//		}
//		/* write the interrupt*/
//		com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
//		(p_bma2x2->dev_addr, BMA2x2_INTR_ENABLE1_REG,
//		&v_data_u8, C_BMA2x2_ONE_U8X);
//		com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
//		(p_bma2x2->dev_addr, BMA2x2_INTR_ENABLE2_REG,
//		&v_data2_u8, C_BMA2x2_ONE_U8X);
//	}
//	return com_rslt;
//}
//
//
///*!
// * @brief This API is used to set
// * the interrupt enable of single tap
// * interrupt in the register 0x19 and 0x1B
// * @note INTR1_sigle_tap -> register 0x19 bit 5
// * @note INTR2_sigle_tap -> register 0x1B bit 5
// *
// *
// *  @param v_channel_u8: The value of single tap interrupt select
// *        v_channel_u8     |   result
// *       ----------------- | ------------------
// *              0          | BMA2x2_ACCEL_INTR1_SINGLE_TAP
// *              1          | BMA2x2_ACCEL_INTR2_SINGLE_TAP
// *
// *  @param v_intr_single_tap_u8: The single tap interrupt enable value
// *       v_intr_single_tap_u8     |   result
// *       ------------------------ | ------------------
// *              0x00              | INTR_DISABLE
// *              0x01              | INTR_ENABLE
// *
// *
// *	@return results of bus communication function
// *	@retval 0 -> Success
// *	@retval -1 -> Error
// *
// *
// */
//s8 bma2x2_set_intr_single_tap(u8 v_channel_u8,
//u8 v_intr_single_tap_u8)
//{
//	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
//		/*  Variable used to return value of
//	communication routine*/
//	s8 com_rslt = ERROR;
//	if (p_bma2x2 == BMA2x2_NULL) {
//		/* Check the struct p_bma2x2 is empty */
//		return E_BMA2x2_NULL_PTR;
//		} else {
//		switch (v_channel_u8) {
//		/* write the single tap value*/
//		case BMA2x2_INTR1_SINGLE_TAP:
//			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
//			(p_bma2x2->dev_addr,
//			BMA2x2_ENABLE_INTR1_PAD_SINGLE_TAP__REG,
//			&v_data_u8, C_BMA2x2_ONE_U8X);
//			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
//			BMA2x2_ENABLE_INTR1_PAD_SINGLE_TAP,
//			v_intr_single_tap_u8);
//			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
//			(p_bma2x2->dev_addr,
//			BMA2x2_ENABLE_INTR1_PAD_SINGLE_TAP__REG,
//			&v_data_u8, C_BMA2x2_ONE_U8X);
//		break;
//		case BMA2x2_INTR2_SINGLE_TAP:
//			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
//			(p_bma2x2->dev_addr,
//			BMA2x2_ENABLE_INTR2_PAD_SINGLE_TAP__REG,
//			&v_data_u8, C_BMA2x2_ONE_U8X);
//			v_data_u8 = BMA2x2_SET_BITSLICE
//			(v_data_u8,
//			BMA2x2_ENABLE_INTR2_PAD_SINGLE_TAP,
//			v_intr_single_tap_u8);
//			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
//			(p_bma2x2->dev_addr,
//			BMA2x2_ENABLE_INTR2_PAD_SINGLE_TAP__REG,
//			&v_data_u8, C_BMA2x2_ONE_U8X);
//		break;
//		default:
//			com_rslt = E_OUT_OF_RANGE;
//		break;
//		}
//	}
//	return com_rslt;
//}
//
//
///*!
// *	@brief This API is used to set
// *	the tap duration in the register 0x2A bit form 0 to 2
// *
// *
// *	@param v_tap_durn_u8: The value of tap duration
// *    v_tap_durn_u8     |    result
// *  --------------------|----------------------
// *     0x00             | TAP_DURN_50_MS
// *     0x01             | TAP_DURN_100_MS
// *     0x02             | TAP_DURN_150_MS
// *     0x03             | TAP_DURN_200_MS
// *     0x04             | TAP_DURN_250_MS
// *     0x05             | TAP_DURN_375_MS
// *     0x06             | TAP_DURN_500_MS
// *     0x07             | TAP_DURN_700_MS
// *
// *
// *
// *	@return results of bus communication function
// *	@retval 0 -> Success
// *	@retval -1 -> Error
// *
// *
// */
//s8 bma2x2_set_tap_durn(u8 v_tap_durn_u8)
//{
//	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
//	/*  Variable used to return value of
//	communication routine*/
//	s8 com_rslt = ERROR;
//	if (p_bma2x2 == BMA2x2_NULL) {
//		/* Check the struct p_bma2x2 is empty */
//		return E_BMA2x2_NULL_PTR;
//		} else {
//			/* write the tap duration */
//			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC(
//			p_bma2x2->dev_addr,
//			BMA2x2_TAP_DURN__REG, &v_data_u8,
//			C_BMA2x2_ONE_U8X);
//			v_data_u8 = BMA2x2_SET_BITSLICE
//			(v_data_u8, BMA2x2_TAP_DURN, v_tap_durn_u8);
//			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
//			(p_bma2x2->dev_addr,
//			BMA2x2_TAP_DURN__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
//		}
//	return com_rslt;
//}
//
//
///*!
// *	@brief This API is used to set
// *	the tap shock form the register 0x2A bit 6
// *
// *
// *
// *	@param v_tap_shock_u8: The value of tap shock
// *    v_tap_shock_u8    |    result
// *  --------------------|----------------------
// *     0x00             | TAP_SHOCK_50_MS
// *     0x01             | TAP_SHOCK_75_MS
// *
// *
// *	@return results of bus communication function
// *	@retval 0 -> Success
// *	@retval -1 -> Error
// *
// *
//*/
//s8 bma2x2_set_tap_shock(u8 v_tap_shock_u8)
//{
//	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
//	/*  Variable used to return value of
//	communication routine*/
//	s8 com_rslt = ERROR;
//	if (p_bma2x2 == BMA2x2_NULL) {
//		/* Check the struct p_bma2x2 is empty */
//		return E_BMA2x2_NULL_PTR;
//		} else {
//			/* write tap shock value*/
//			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
//			(p_bma2x2->dev_addr,
//			BMA2x2_TAP_SHOCK_DURN__REG, &v_data_u8,
//			C_BMA2x2_ONE_U8X);
//			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
//			BMA2x2_TAP_SHOCK_DURN, v_tap_shock_u8);
//			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
//			(p_bma2x2->dev_addr,
//			BMA2x2_TAP_SHOCK_DURN__REG, &v_data_u8,
//			C_BMA2x2_ONE_U8X);
//		}
//	return com_rslt;
//}
//
//
///*!
// *	@brief This API is used to set
// *	the tap quiet in the register 0x2A bit 7
// *
// *
// *
// *  @param  v_tap_quiet_u8 : The value of tap quiet
// *    v_tap_quiet_u8    |    result
// *  --------------------|----------------------
// *     0x00             | TAP_QUIET_30_MS
// *     0x01             | TAP_QUIET_20_MS
// *
// *	@return results of bus communication function
// *	@retval 0 -> Success
// *	@retval -1 -> Error
// *
// *
// */
//s8 bma2x2_set_tap_quiet(u8 v_tap_quiet_u8)
//{
//	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
//		/*  Variable used to return value of
//	communication routine*/
//	s8 com_rslt = ERROR;
//	if (p_bma2x2 == BMA2x2_NULL) {
//		/* Check the struct p_bma2x2 is empty */
//		return E_BMA2x2_NULL_PTR;
//		} else {
//			/* write the tap quiet value*/
//			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
//			(p_bma2x2->dev_addr,
//			BMA2x2_TAP_QUIET_DURN__REG,
//			&v_data_u8, C_BMA2x2_ONE_U8X);
//			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
//			BMA2x2_TAP_QUIET_DURN, v_tap_quiet_u8);
//			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
//			(p_bma2x2->dev_addr,
//			BMA2x2_TAP_QUIET_DURN__REG,
//			&v_data_u8, C_BMA2x2_ONE_U8X);
//		}
//	return com_rslt;
//}
//
///*!
// *	@brief This API is used to set
// *	the tap threshold in the register 0x2B bit from 0 to 4
// *
// *
// *
// *  @param v_tap_thres_u8 : The value of tap threshold
// *	@note Tap threshold of single and double tap corresponding to accel range
// *     range            |    Tap threshold
// *  --------------------|----------------------
// *     2g               | (v_tap_thres_u8 * 62.5)mg
// *     4g               | (v_tap_thres_u8 * 125)mg
// *     8g               | (v_tap_thres_u8 * 250)mg
// *     16g              | (v_tap_thres_u8 * 500)mg
// *
// *
// *	@return results of bus communication function
// *	@retval 0 -> Success
// *	@retval -1 -> Error
// *
// *
// */
//s8 bma2x2_set_tap_thres(u8 v_tap_thres_u8)
//{
//	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
//	/*  Variable used to return value of
//	communication routine*/
//	s8 com_rslt = ERROR;
//	if (p_bma2x2 == BMA2x2_NULL) {
//		/* Check the struct p_bma2x2 is empty */
//		return E_BMA2x2_NULL_PTR;
//		} else {
//			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
//			(p_bma2x2->dev_addr,
//			BMA2x2_TAP_THRES__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
//			v_data_u8 = BMA2x2_SET_BITSLICE
//			(v_data_u8, BMA2x2_TAP_THRES, v_tap_thres_u8);
//			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
//			(p_bma2x2->dev_addr,
//			BMA2x2_TAP_THRES__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
//		}
//	return com_rslt;
//}
