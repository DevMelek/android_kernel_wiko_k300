//s_add new sensor driver here
//export funtions
/*GC*/
UINT32 GC5025_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
UINT32 GC5025_MIPI_RAW_SensorInit_CMK(PSENSOR_FUNCTION_STRUCT *pfFunc);
/*S5K*/
UINT32 S5K4H8_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
UINT32 S5K4H8_MIPI_RAW_SensorInit_Holitech(PSENSOR_FUNCTION_STRUCT *pfFunc);
UINT32 S5K3H7YX_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
/*OV*/
UINT32 OV8856_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
/*Others*/


//! Add Sensor Init function here
//! Note:
//! 1. Add by the resolution from ""large to small"", due to large sensor
//!    will be possible to be main sensor.
//!    This can avoid I2C error during searching sensor.
//! 2. This file should be the same as mediatek\custom\common\hal\imgsensor\src\sensorlist.cpp
ACDK_KD_SENSOR_INIT_FUNCTION_STRUCT kdSensorList[MAX_NUM_OF_SUPPORT_SENSOR+1] =
{
#if defined(GC5025_MIPI_RAW_SUNWIN)
    {GC5025_SUNWIN_SENSOR_ID, SENSOR_DRVNAME_GC5025_MIPI_RAW_SUNWIN_K400, GC5025_MIPI_RAW_SensorInit},
#endif
#if defined(S5K4H8_MIPI_RAW_SUNWIN)
    {S5K4H8_SUNWIN_SENSOR_ID, SENSOR_DRVNAME_S5K4H8_MIPI_RAW_SUNWIN_K400, S5K4H8_MIPI_RAW_SensorInit},
#endif
#if defined(OV8856_MIPI_RAW_SUNWIN)
    {OV8856_SUNWIN_SENSOR_ID, SENSOR_DRVNAME_OV8856_MIPI_RAW_SUNWIN_K400, OV8856_MIPI_RAW_SensorInit},
#endif
#if defined(GC5025_MIPI_RAW_CMK)
    {GC5025_CMK_SENSOR_ID, SENSOR_DRVNAME_GC5025_MIPI_RAW_CMK_K400, GC5025_MIPI_RAW_SensorInit_CMK},
#endif
#if defined(S5K4H8_MIPI_RAW_HOLITECH)
    {S5K4H8_HOLITECH_SENSOR_ID, SENSOR_DRVNAME_S5K4H8_MIPI_RAW_HOLITECH_K400, S5K4H8_MIPI_RAW_SensorInit_Holitech},
#endif
#if defined(S5K3H7YX_MIPI_RAW)
    {S5K3H7YX_SENSOR_ID, SENSOR_DRVNAME_S5K3H7YX_MIPI_RAW, S5K3H7YX_MIPI_RAW_SensorInit},
#endif
/*  ADD sensor driver before this line */
    {0,{0},NULL}, //end of list
};
//e_add new sensor driver here
