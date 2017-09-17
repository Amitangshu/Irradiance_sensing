COMPONENT=MVizAppC
CFLAGS += -DLOW_POWER_LISTENING
CFLAGS += -DLPL_DEF_LOCAL_WAKEUP=128
CFLAGS += -DLPL_DEF_REMOTE_WAKEUP=128
CFLAGS += -DDELAY_AFTER_RECEIVE=20

CFLAGS += -DCC2420_NO_ADDRESS_RECOGNITION
CFLAGS += -DCC2420_DEF_CHANNEL=26

# Location for printf.h
CFLAGS += -I$(TOSDIR)/lib/printf
# Location for MDA300CA files
#CFLAGS += -I$(TOSDIR)/sensorboards/mda300ca/ -I
# Location for sht11 files
CFLAGS += -I$(TOSDIR)/chips/sht11/
CFLAGS += -I$(TOSDIR)/lib/net/ -I$(TOSDIR)/lib/net/ctp -I$(TOSDIR)/lib/net/4bitle -I$(TOSDIR)/sensorboards/mts300 -I$(TOSDIR)/sensorboards/mda300 -I.
# Parameters for floating point math.h functions and floating point printf
CFLAGS += -Wl,-lm -Wl,-lprintf_flt  -Wl,-L"libm.a" -Wl,-L"libprintf_flt.a"  -Wl,-u,vfprintf -lprintf_flt -lm

BUILD_EXTRA_DEPS = MVizMsg.class

MVizMsg.java: MViz.h
	mig -target=null -java-classname=MVizMsg java MViz.h mviz_msg -o $@

MVizMsg.class: MVizMsg.java
	javac MVizMsg.java

include $(MAKERULES)
