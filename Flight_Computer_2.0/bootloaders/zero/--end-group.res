13
build/board_driver_i2c.o 0
build/board_driver_led.o 4
187 ebc817ec PREVAILING_DEF_IRONLY LED_pulse
199 ebc817ec PREVAILING_DEF_IRONLY ledKeepValue
211 ebc817ec PREVAILING_DEF_IRONLY ledDirection
214 ebc817ec PREVAILING_DEF_IRONLY ledTargetValue
build/board_driver_pmic.o 0
build/board_driver_jtag.o 0
build/board_driver_serial.o 7
1308 ebc810fb PREVAILING_DEF_IRONLY uart_basic_init
1312 ebc810fb PREVAILING_DEF_IRONLY uart_disable
1317 ebc810fb PREVAILING_DEF_IRONLY uart_write_byte
1321 ebc810fb PREVAILING_DEF_IRONLY uart_read_byte
1336 ebc810fb PREVAILING_DEF_IRONLY uart_drv_error_flag
1327 ebc810fb PREVAILING_DEF_IRONLY uart_write_buffer_polled
1329 ebc810fb PREVAILING_DEF_IRONLY uart_read_buffer_polled
build/board_driver_usb.o 14
1407 ebc86a21 PREVAILING_DEF_IRONLY USB_IsConfigured
1501 ebc86a21 PREVAILING_DEF_IRONLY usb_endpoint_table
1496 ebc86a21 PREVAILING_DEF_IRONLY udd_ep_out_cache_buffer
1505 ebc86a21 PREVAILING_DEF_IRONLY udd_ep_in_cache_buffer
1418 ebc86a21 PREVAILING_DEF_IRONLY USB_Open
1421 ebc86a21 PREVAILING_DEF_IRONLY USB_Init
1428 ebc86a21 PREVAILING_DEF_IRONLY USB_Write
1434 ebc86a21 PREVAILING_DEF_IRONLY USB_Read
1436 ebc86a21 PREVAILING_DEF_IRONLY USB_Read_blocking
1441 ebc86a21 PREVAILING_DEF_IRONLY USB_SendStall
1445 ebc86a21 PREVAILING_DEF_IRONLY USB_SendZlp
1450 ebc86a21 PREVAILING_DEF_IRONLY USB_SetAddress
1452 ebc86a21 PREVAILING_DEF_IRONLY USB_Configure
1456 ebc86a21 RESOLVED_IR sam_ba_usb_CDC_Enumerate
build/board_init.o 2
310 ebc84d97 PREVAILING_DEF_IRONLY board_init
319 ebc84d97 PREVAILING_DEF_IRONLY g_interrupt_enabled
build/board_startup.o 14
187 ebc8263a PREVAILING_DEF_IRONLY NMI_Handler
189 ebc8263a PREVAILING_DEF_IRONLY HardFault_Handler
191 ebc8263a PREVAILING_DEF_IRONLY SVC_Handler
193 ebc8263a PREVAILING_DEF_IRONLY PendSV_Handler
195 ebc8263a PREVAILING_DEF_IRONLY_EXP Reset_Handler
283 ebc8263a PREVAILING_DEF_IRONLY exception_table
212 ebc8263a RESOLVED_EXEC __data_start__
214 ebc8263a RESOLVED_EXEC __data_end__
216 ebc8263a RESOLVED_EXEC __etext
218 ebc8263a RESOLVED_EXEC __bss_start__
220 ebc8263a RESOLVED_EXEC __bss_end__
285 ebc8263a UNDEF __StackTop
197 ebc8263a RESOLVED_IR SysTick_Handler
200 ebc8263a RESOLVED_IR main
build/main.o 12
1433 ebc81a29 PREVAILING_DEF_IRONLY pulSketch_Start_Address
319 ebc81a29 PREVAILING_DEF main
321 ebc81a29 PREVAILING_DEF_IRONLY SysTick_Handler
1429 ebc81a29 UNDEF __sketch_vectors_ptr
323 ebc81a29 RESOLVED_IR sam_ba_monitor_run
327 ebc81a29 RESOLVED_IR sam_ba_monitor_init
329 ebc81a29 RESOLVED_IR serial_sharp_received
1418 ebc81a29 RESOLVED_IR usb_init
1420 ebc81a29 RESOLVED_IR serial_open
1422 ebc81a29 RESOLVED_IR board_init
1424 ebc81a29 RESOLVED_IR sam_ba_monitor_sys_tick
1427 ebc81a29 RESOLVED_IR LED_pulse
build/sam_ba_usb.o 13
1355 ebc80046 PREVAILING_DEF_IRONLY usb_init
1415 ebc80046 PREVAILING_DEF_IRONLY sam_ba_cdc
1361 ebc80046 PREVAILING_DEF_IRONLY USB_SendString
1365 ebc80046 PREVAILING_DEF_IRONLY sam_ba_usb_CDC_Enumerate
1440 ebc80046 RESOLVED_IR udd_ep_out_cache_buffer
1417 ebc80046 RESOLVED_IR line_coding
1370 ebc80046 RESOLVED_IR USB_Open
1373 ebc80046 RESOLVED_IR USB_Init
1380 ebc80046 RESOLVED_IR USB_Write
1402 ebc80046 RESOLVED_IR USB_SendStall
1406 ebc80046 RESOLVED_IR USB_SendZlp
1408 ebc80046 RESOLVED_IR USB_Configure
1413 ebc80046 RESOLVED_IR USB_SetAddress
build/sam_ba_cdc.o 11
1304 ebc82803 PREVAILING_DEF_IRONLY cdc_putc
1307 ebc82803 PREVAILING_DEF_IRONLY cdc_getc
1315 ebc82803 PREVAILING_DEF_IRONLY cdc_is_rx_ready
1320 ebc82803 PREVAILING_DEF_IRONLY cdc_write_buf
1325 ebc82803 PREVAILING_DEF_IRONLY cdc_read_buf
1327 ebc82803 PREVAILING_DEF_IRONLY cdc_read_buf_xmd
1373 ebc82803 PREVAILING_DEF_IRONLY line_coding
1357 ebc82803 RESOLVED_IR sam_ba_cdc
1339 ebc82803 RESOLVED_IR USB_Read
1348 ebc82803 RESOLVED_IR USB_Write
1355 ebc82803 RESOLVED_IR USB_IsConfigured
build/sam_ba_monitor.o 43
398 ebc81ce1 PREVAILING_DEF_IRONLY erased_from
400 ebc81ce1 PREVAILING_DEF_IRONLY PAGE_SIZE
402 ebc81ce1 PREVAILING_DEF_IRONLY MAX_FLASH
414 ebc81ce1 PREVAILING_DEF_IRONLY txLEDPulse
442 ebc81ce1 PREVAILING_DEF_IRONLY ptr_monitor_if
444 ebc81ce1 PREVAILING_DEF_IRONLY rxLEDPulse
447 ebc81ce1 PREVAILING_DEF_IRONLY sp
308 ebc81ce1 PREVAILING_DEF_IRONLY sam_ba_monitor_init
450 ebc81ce1 PREVAILING_DEF_IRONLY uart_if
454 ebc81ce1 PREVAILING_DEF_IRONLY b_sam_ba_interface_usart
456 ebc81ce1 PREVAILING_DEF_IRONLY usbcdc_if
313 ebc81ce1 PREVAILING_DEF_IRONLY sam_ba_putdata_term
458 ebc81ce1 PREVAILING_DEF_IRONLY b_terminal_mode
315 ebc81ce1 PREVAILING_DEF_IRONLY call_applet
460 ebc81ce1 PREVAILING_DEF_IRONLY b_security_enabled
395 ebc81ce1 PREVAILING_DEF_IRONLY data
462 ebc81ce1 PREVAILING_DEF_IRONLY length
464 ebc81ce1 PREVAILING_DEF_IRONLY ptr
466 ebc81ce1 PREVAILING_DEF_IRONLY i
468 ebc81ce1 PREVAILING_DEF_IRONLY command
393 ebc81ce1 PREVAILING_DEF_IRONLY current_number
470 ebc81ce1 PREVAILING_DEF_IRONLY u32tmp
472 ebc81ce1 PREVAILING_DEF_IRONLY ptr_data
474 ebc81ce1 PREVAILING_DEF_IRONLY j
391 ebc81ce1 PREVAILING_DEF_IRONLY RomBOOT_Version
388 ebc81ce1 PREVAILING_DEF_IRONLY RomBOOT_ExtendedCapabilities
320 ebc81ce1 PREVAILING_DEF_IRONLY sam_ba_monitor_sys_tick
322 ebc81ce1 PREVAILING_DEF_IRONLY sam_ba_monitor_run
500 ebc81ce1 PREVAILING_DEF_IRONLY PAGES
326 ebc81ce1 RESOLVED_IR cdc_putc
329 ebc81ce1 RESOLVED_IR cdc_getc
337 ebc81ce1 RESOLVED_IR cdc_is_rx_ready
339 ebc81ce1 RESOLVED_IR cdc_write_buf
341 ebc81ce1 RESOLVED_IR cdc_read_buf
343 ebc81ce1 RESOLVED_IR cdc_read_buf_xmd
345 ebc81ce1 RESOLVED_IR serial_putc
348 ebc81ce1 RESOLVED_IR serial_getc
351 ebc81ce1 RESOLVED_IR serial_is_rx_ready
353 ebc81ce1 RESOLVED_IR serial_putdata
355 ebc81ce1 RESOLVED_IR serial_getdata
357 ebc81ce1 RESOLVED_IR serial_putdata_xmd
359 ebc81ce1 RESOLVED_IR serial_getdata_xmd
364 ebc81ce1 RESOLVED_IR serial_add_crc
build/sam_ba_serial.o 26
1311 ebc85a13 PREVAILING_DEF_IRONLY serial_open
1395 ebc85a13 PREVAILING_DEF_IRONLY b_sharp_received
1397 ebc85a13 PREVAILING_DEF_IRONLY idx_rx_read
1399 ebc85a13 PREVAILING_DEF_IRONLY idx_rx_write
1401 ebc85a13 PREVAILING_DEF_IRONLY idx_tx_read
1403 ebc85a13 PREVAILING_DEF_IRONLY idx_tx_write
1405 ebc85a13 PREVAILING_DEF_IRONLY error_timeout
1313 ebc85a13 PREVAILING_DEF_IRONLY serial_close
1317 ebc85a13 PREVAILING_DEF_IRONLY serial_putc
1325 ebc85a13 PREVAILING_DEF_IRONLY serial_is_rx_ready
1328 ebc85a13 PREVAILING_DEF_IRONLY serial_getc
1333 ebc85a13 PREVAILING_DEF_IRONLY serial_sharp_received
1335 ebc85a13 PREVAILING_DEF_IRONLY serial_readc
1413 ebc85a13 PREVAILING_DEF_IRONLY buffer_rx_usart
1340 ebc85a13 PREVAILING_DEF_IRONLY serial_putdata
1345 ebc85a13 PREVAILING_DEF_IRONLY serial_getdata
1350 ebc85a13 PREVAILING_DEF_IRONLY serial_add_crc
1424 ebc85a13 PREVAILING_DEF_IRONLY size_of_data
1426 ebc85a13 PREVAILING_DEF_IRONLY mode_of_transfer
1357 ebc85a13 PREVAILING_DEF_IRONLY serial_putdata_xmd
1373 ebc85a13 PREVAILING_DEF_IRONLY serial_getdata_xmd
1428 ebc85a13 PREVAILING_DEF_IRONLY buffer_tx_usart
1379 ebc85a13 RESOLVED_IR uart_basic_init
1383 ebc85a13 RESOLVED_IR uart_disable
1388 ebc85a13 RESOLVED_IR uart_write_byte
1392 ebc85a13 RESOLVED_IR uart_read_byte
