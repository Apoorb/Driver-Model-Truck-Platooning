Platooning_ent_ramp_source_edit_03_20_17

CACC vehicles can detect manual vehicles on hte ramp and slow dowm or change lane to facilitate merging
Controlling manual vehicles using dll to obtain the information of vehicles on the ramp

Following the following coding convention in VISSIM for the platooning ramp source codes to work:
Link No:
1-100:  Basic Freeway Section
501-600: An auxilary lane is present on the right
801-900: Freeway section before and after an entrance ramp with no auxilary lane
901-1000: Freeway section after an entrance ramp- containing a small auxilary lane (<300ft)
1201-1300: Entrance Ramp links. (Ramp sections with no auxilary lane)

Desired Speed:
Manual Vehicles have a desired speed at the start of the link greater than or equal to 75 mph 
Connected vehicles have a desired speed less than 75 mph
Need to put a desired speed decision immediately after the vehicle input for this code to work 