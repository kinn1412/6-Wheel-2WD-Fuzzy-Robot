# L4 — Control algorithms. PURE logic, NO hardware dependency on purpose.
# (No REQUIRES of hal/driver -> including hal_*.h here is a build error.)
# This lets the whole component compile & unit-test on the host PC.
idf_component_register(
    SRCS "src/pid.c" "src/controller_pid.c"
    INCLUDE_DIRS "include"
)
