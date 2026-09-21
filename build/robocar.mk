################################################################################
# robocar.mk
#
# Replaces build_make/mtkernel_3/app_program/subdir.mk inside the
# mtk3smp-rp2040 port. build/setup.sh copies it into place.
#
# The port's stock subdir.mk compiles $(wildcard ../app_program/*.c) — a flat
# directory. This project keeps its sources three levels up, in the repo that
# contains the port as a submodule:
#
#     <repo>/core, drivers, subsystems, app     <- our code
#     <repo>/external/mtk3smp-rp2040/build_make <- where make runs
#
# so this file points the same compile rule at those four directories and
# puts the objects under build_make/mtkernel_3/robocar/. The port's own demo
# in app_program/ is simply no longer compiled; usermain() comes from
# app/app_main.c and overrides the kernel's weak default.
#
# Nothing here changes INCPATH or CFLAGS for the rest of the tree.
################################################################################

RC_ROOT := ../../..
RC_DIRS := core drivers subsystems app

RC_SRCS := $(foreach d,$(RC_DIRS),$(wildcard $(RC_ROOT)/$(d)/*.c))
RC_OBJS := $(patsubst $(RC_ROOT)/%.c,mtkernel_3/robocar/%.o,$(RC_SRCS))
# The port keeps the device-driver API headers (dev_i2c.h, dev_adc.h) in
# device/include, which its own INCPATH never lists.
RC_INC  := $(foreach d,$(RC_DIRS),-I"$(RC_ROOT)/$(d)") -I"../device/include"

OBJS   += $(RC_OBJS)
C_DEPS += $(RC_OBJS:.o=.d)

$(shell mkdir -p $(addprefix mtkernel_3/robocar/,$(RC_DIRS)))

mtkernel_3/robocar/%.o: $(RC_ROOT)/%.c
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) $(RC_INC) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
