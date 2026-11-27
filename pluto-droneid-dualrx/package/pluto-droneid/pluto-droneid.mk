PLUTO_DRONEID_VERSION = 1.0
PLUTO_DRONEID_SITE = $(BR2_EXTERNAL_PLUTODRONEID_PATH)
PLUTO_DRONEID_SITE_METHOD = local
PLUTO_DRONEID_DEPENDENCIES = libiio

define PLUTO_DRONEID_BUILD_CMDS
	$(TARGET_MAKE_ENV) $(MAKE) -C $(@D)/app \
		CC="$(TARGET_CC)" CXX="$(TARGET_CXX)" \
		CFLAGS="$(TARGET_CFLAGS)" CXXFLAGS="$(TARGET_CXXFLAGS)" \
		LDFLAGS="$(TARGET_LDFLAGS)" all
endef

define PLUTO_DRONEID_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/app/pluto-droneid-tx \
		$(TARGET_DIR)/usr/bin/pluto-droneid-tx
	$(INSTALL) -D -m 0755 $(@D)/app/dji-droneid-tx \
		$(TARGET_DIR)/usr/bin/dji-droneid-tx
	$(INSTALL) -D -m 0755 $(@D)/app/pluto-droneid-count \
		$(TARGET_DIR)/usr/bin/pluto-droneid-count
	$(INSTALL) -D -m 0755 $(@D)/app/pluto-droneid-capture \
		$(TARGET_DIR)/usr/bin/pluto-droneid-capture
	$(INSTALL) -D -m 0755 $(@D)/app/pluto-droneid-scan \
		$(TARGET_DIR)/usr/bin/pluto-droneid-scan
	$(INSTALL) -D -m 0755 $(@D)/service/pluto-droneid \
		$(TARGET_DIR)/usr/bin/pluto-droneid
	$(INSTALL) -D -m 0755 $(@D)/service/S70pluto-droneid \
		$(TARGET_DIR)/etc/init.d/S70pluto-droneid
	$(INSTALL) -D -m 0644 $(@D)/service/pluto-droneid.conf \
		$(TARGET_DIR)/etc/pluto-droneid.conf
	$(INSTALL) -D -m 0644 $(@D)/examples/test-packet-30m72.cf32 \
		$(TARGET_DIR)/usr/share/pluto-droneid/test-packet-30m72.cf32
	$(INSTALL) -D -m 0644 $(@D)/examples/test-packet-15m36.cf32 \
		$(TARGET_DIR)/usr/share/pluto-droneid/test-packet-15m36.cf32
	$(INSTALL) -D -m 0644 $(@D)/examples/test-payload.bin \
		$(TARGET_DIR)/usr/share/pluto-droneid/test-payload.bin
	$(INSTALL) -D -m 0644 $(@D)/examples/test-packet-30m72.json \
		$(TARGET_DIR)/usr/share/pluto-droneid/test-packet-30m72.json
	$(INSTALL) -D -m 0644 $(@D)/examples/test-packet-15m36.json \
		$(TARGET_DIR)/usr/share/pluto-droneid/test-packet-15m36.json
endef

$(eval $(generic-package))
