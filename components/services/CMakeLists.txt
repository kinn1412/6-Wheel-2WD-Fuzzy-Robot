# L3 — Middleware: shared state (seqlock), odometry, protocol, safety.
idf_component_register(
    SRCS "src/odometry.c" "src/protocol.c" "src/safety.c"
    INCLUDE_DIRS "include"
    REQUIRES bsp
    PRIV_REQUIRES esp_timer
)
