#ifndef __DHT11_H__
#define __DHT11_H__

#define DHT11_PIN	22

/*! Error codes */
typedef enum {
  DHT11_OK, /*!< OK */
  DHT11_NO_PULLUP, /*!< no pull-up present */
  DHT11_NO_ACK_0, /*!< no 0 acknowledge detected */
  DHT11_NO_ACK_1, /*!< no 1 acknowledge detected */
  DHT11_NO_DATA_0, /*!< low level expected during data transmission */
  DHT11_NO_DATA_1, /*!< high level expected during data transmission */
  DHT11_BAD_CRC,   /*!< bad CRC */
} DHTxx_ErrorCode;

DHTxx_ErrorCode DHTxx_Read(uint16_t *temperatureCentigrade, uint16_t *humidityCentipercent);

#endif