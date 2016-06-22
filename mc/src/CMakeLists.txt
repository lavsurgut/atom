##########################################################################
# "THE ANY BEVERAGE-WARE LICENSE" (Revision 42 - based on beer-ware
# license):
# <dev@layer128.net> wrote this file. As long as you retain this notice
# you can do whatever you want with this stuff. If we meet some day, and
# you think this stuff is worth it, you can buy me a be(ve)er(age) in
# return. (I don't like beer much.)
#
# Matthias Kleemann
##########################################################################



#add_avr_library(
#TWI_Master
#TWI_Master.c
#TWI_Master.h
#)

add_avr_executable(
   atom
   #debug.c
   #avr_sample.c
   main.c
)

#avr_target_link_libraries(avr_sample TWI_Master)
