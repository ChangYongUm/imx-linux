# Overlays for the TechNexion platform

dtbo-$(CONFIG_ARCH_MXC) += imx8mm-edm-g-wb-sn65dsi84-vl10112880.dtbo imx8mm-edm-g-wb-sn65dsi84-vl15613676.dtbo imx8mm-edm-g-wb-sn65dsi84-vl215192108.dtbo \
			   imx8mm-edm-g-wb-hdmi2mipi-tc358743.dtbo imx8mm-edm-g-wb-tevs.dtbo imx8mm-edm-g-wb-vls.dtbo

dtbo-$(CONFIG_ARCH_MXC) += imx8mp-edm-g-wb-lvds-vl10112880.dtbo imx8mp-edm-g-wb-lvds-vl15613676.dtbo imx8mp-edm-g-wb-lvds-vl156192108.dtbo \
				imx8mp-edm-g-wb-lvds-vl215192108.dtbo imx8mp-edm-g-wb-tevs.dtbo

targets += dtbs dtbs_install
targets += $(dtbo-y)

always-y        := $(dtbo-y)
clean-files     := *.dtbo
