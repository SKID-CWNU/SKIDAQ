#include <Arduino.h>

#define SENSOR_BUFF_SIZE                    (200u)

typedef struct _Sensor_Data_s
{
  uint8_t temperature[SENSOR_BUFF_SIZE];
  uint8_t humidity[SENSOR_BUFF_SIZE];
  uint8_t sensor_idx;
} Sensor_Data_s;

Sensor_Data_s * Get_TemperatureAndHumidity( void );

#endif /* INC_MAIN_H_ */