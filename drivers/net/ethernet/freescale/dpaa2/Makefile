# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Freescale DPAA2 Ethernet controller
#

obj-$(CONFIG_FSL_DPAA2_ETH)		+= fsl-dpaa2-eth.o
obj-$(CONFIG_FSL_DPAA2_PTP_CLOCK)	+= fsl-dpaa2-ptp.o
obj-$(CONFIG_FSL_DPAA2_SWITCH)		+= fsl-dpaa2-switch.o
obj-$(CONFIG_FSL_DPAA2_MAC)		+= fsl-dpaa2-mac.o

fsl-dpaa2-eth-objs	:= dpaa2-eth.o dpaa2-ethtool.o dpni.o dpaa2-mac.o dpmac.o dpaa2-eth-devlink.o dpaa2-xsk.o
fsl-dpaa2-eth-${CONFIG_FSL_DPAA2_ETH_DCB} += dpaa2-eth-dcb.o
fsl-dpaa2-eth-${CONFIG_DEBUG_FS} += dpaa2-eth-debugfs.o
fsl-dpaa2-eth-${CONFIG_FSL_DPAA2_ETH_CEETM} += dpaa2-eth-ceetm.o
fsl-dpaa2-eth-${CONFIG_MACSEC} += dpaa2-eth-macsec.o
fsl-dpaa2-ptp-objs	:= dpaa2-ptp.o dprtc.o
fsl-dpaa2-switch-objs	:= dpaa2-switch.o dpaa2-switch-ethtool.o dpsw.o dpaa2-switch-flower.o dpaa2-mac.o dpmac.o
fsl-dpaa2-mac-objs	:= dpaa2-mac-standalone.o dpaa2-mac.o dpmac.o

# Needed by the tracing framework
CFLAGS_dpaa2-eth.o := -I$(src)
