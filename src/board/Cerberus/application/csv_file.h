/*
 * csv_file.h
 *
 *  Created on: 5 апр. 2023 г.
 *      Author: Install
 */

#ifndef CSV_FILE_H_
#define CSV_FILE_H_

uint16_t sd_parse_to_bytes_pack1(char *buffer, pack1_t *pack1);

uint16_t sd_parse_to_bytes_pack2(char *buffer, pack2_t *pack2);

uint16_t sd_parse_to_bytes_pack3(char *buffer, pack3_t *pack3);

uint16_t sd_parse_to_bytes_pack4(char *buffer, pack4_t *pack4);


#endif /* CSV_FILE_H_ */
