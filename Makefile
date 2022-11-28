MAKE := make
all:
	$(MAKE) -C Portable/board_budgetcan_g0/
	$(MAKE) -C Portable/board_canablev1/
	$(MAKE) -C Portable/board_canablev2/
	$(MAKE) -C Portable/board_DevEBoxH7/
