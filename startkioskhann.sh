scripts/start-kiosk.sh -d -l range \
  --iwr6843 --iwr6843-self-trigger --trigger sound \
  --iwr6843-config config/iwr6843_l3dump_adaptive_47f3ms_53bin_a16_hann.cfg \
  --iwr6843-tee-m 1.845 \
  --log-dir "$(pwd)/openflight_sessions"
