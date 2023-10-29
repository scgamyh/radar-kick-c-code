AD936X_IIOSTREAM_SWITCH_VERSION:= 1.0.0
AD936X_IIOSTREAM_SWITCH_SITE:= $(TOPDIR)/package/ad936x_iiostream_switch
AD936X_IIOSTREAM_SWITCH_SITE_METHOD=local
AD936X_IIOSTREAM_SWITCH_INSTALL_TARGET=YES
AD936X_IIOSTREAM_SWITCH_DEPENDENCIES = libiio 
 
define AD936X_IIOSTREAM_SWITCH_BUILD_CMDS
	  $(TARGET_CC) $(TARGET_CFLAGS) $(TARGET_LDFLAGS) \
                $(@D)/main.c -o $(@D)/ad936x_iiostream_switch -lm -lpthread -liio 
endef
 
define AD936X_IIOSTREAM_SWITCH_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/ad936x_iiostream_switch $(TARGET_DIR)/bin
endef
 
define AD936X_IIOSTREAM_SWITCH_PERMISSIONS
	/bin/ad936x_iiostream_switch f 4755 0 0 - - - - - 
endef
 
$(eval $(generic-package))

