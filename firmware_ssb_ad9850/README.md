See https://rfcorner.in/posts/the-polar-express/ for a demo.

Signal path:

```
Audio → 120Hz HPF → 257-tap Hilbert → angle diff → gate (kill nulls) → AD9850 freq + PWM amp
```

```
$ elf-size-analyze -R build-output/usb_analog_microphone.elf
======================================== RAM =========================================
Symbol                                                                    Size   %
======================================================================================
/                                                                         8054   99.95
  workspace                                                               7334   91.02
    pico-sdk                                                              5044   62.60
      src                                                                 2932   36.39
        rp2_common                                                        2512   31.17
          pico_stdio_usb                                                    92    1.14
            stdio_usb_descriptors.c                                         57    0.71
              desc_str.0                                                    40    0.50
              usbd_serial_str                                               17    0.21
            stdio_usb.c                                                     34    0.42
              one_shot_timer_crit_sec                                        8    0.10
              stdio_usb_mutex                                                8    0.10
              last_avail_time.1                                              8    0.10
              chars_available_callback                                       4    0.05
              chars_available_param                                          4    0.05
              one_shot_timer_pending                                         1    0.01
              low_priority_irq_num                                           1    0.01
            reset_interface.c                                                1    0.01
              itf_num                                                        1    0.01
          pico_multicore/multicore.c                                      2048   25.42
            core1_stack                                                   2048   25.42
          pico_runtime_init/runtime_init.c                                 272    3.38
            ram_vector_table                                               272    3.38
          hardware_clocks/clocks.c                                          40    0.50
            configured_freq                                                 40    0.50
          hardware_sync_spin_lock/include/hardware/sync/spin_lock.h         32    0.40
            _sw_spin_locks                                                  32    0.40
          pico_stdio/stdio.c                                                 8    0.10
            drivers                                                          4    0.05
            filter                                                           4    0.05
          pico_unique_id/unique_id.c                                         8    0.10
            retrieved_id                                                     8    0.10
          hardware_watchdog/watchdog.c                                       4    0.05
            load_value                                                       4    0.05
          pico_printf/printf.c                                               4    0.05
            lazy_vsnprintf                                                   4    0.05
          hardware_irq/irq.c                                                 2    0.02
            irq_handler_chain_free_slot_head                                 1    0.01
            user_irq_claimed                                                 1    0.01
          hardware_timer/timer.c                                             2    0.02
            claimed                                                          2    0.02
        common/pico_time/time.c                                            420    5.21
          default_alarm_pool_entries                                       384    4.77
          pools                                                             32    0.40
          sleep_notifier                                                     4    0.05
      lib/tinyusb/src                                                     2112   26.21
        class                                                             1072   13.30
          vendor/vendor_device.c                                           724    8.98
            _vendord_itf                                                   596    7.40
            _vendord_epbuf                                                 128    1.59
          cdc/cdc_device.c                                                 348    4.32
            _cdcd_itf                                                      220    2.73
            _cdcd_epbuf                                                    128    1.59
        device                                                             393    4.88
          usbd.c                                                           309    3.83
            _usbd_qdef_buf                                                 192    2.38
            _usbd_dev                                                       87    1.08
            _ubsd_mutexdef                                                   8    0.10
            _usbd_spin                                                       8    0.10
            _app_driver                                                      4    0.05
            _usbd_mutex                                                      4    0.05
            _usbd_q                                                          4    0.05
            _app_driver_count                                                1    0.01
            _usbd_queued_setup                                               1    0.01
          usbd_control.c                                                    84    1.04
            _ctrl_epbuf                                                     64    0.79
            _ctrl_xfer                                                      20    0.25
        portable/raspberrypi/rp2040/dcd_rp2040.c                           645    8.00
          hw_endpoints                                                     640    7.94
          hw_buffer_ptr                                                      4    0.05
          _sof_enable                                                        1    0.01
        tusb.c                                                               2    0.02
          _tusb_rhport_role                                                  2    0.02
    src                                                                   2290   28.42
      main.cpp                                                            1102   13.68
        cat_buf                                                           1024   12.71
        sample_buffer                                                       64    0.79
        cat_idx                                                              4    0.05
        phase_delay_us                                                       4    0.05
        samples_read                                                         4    0.05
        use_melody_test                                                      1    0.01
        use_two_tone_test                                                    1    0.01
      polar_mod.cpp                                                       1072   13.30
        hilbert_history                                                   1028   12.76
        delay_bp                                                            32    0.40
        last_angle                                                           4    0.05
        comp_env                                                             4    0.05
        hilbert_idx                                                          4    0.05
      arduino_compat.cpp                                                    96    1.19
        Wire1                                                               48    0.60
        Wire                                                                48    0.60
      ad9850.cpp                                                            20    0.25
        ad9850_base_freq                                                     8    0.10
        ad9850_current_freq                                                  4    0.05
        ad9850_tuning_word                                                   4    0.05
        ad9850_offset                                                        4    0.05
  build/newlib-38V0JC/newlib-4.4.0.20231231/newlib/libc/include/sys/...    720    8.94
    __atexit0                                                              400    4.96
    __sf                                                                   312    3.87
    __stdio_exit_handler                                                     4    0.05
    __atexit                                                                 4    0.05
?                                                                            4    0.05
  claimed                                                                    4    0.05
======================================================================================
Symbols total                                                             8058
======================================================================================
```
