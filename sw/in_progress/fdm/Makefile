# 
#   $Id$
#   Copyright (C) 2004 Pascal Brisset, Antoine Drouin
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.  
# 

# Quiet compilation
Q=@

all: fdm_step

fdm_step: fdm_step.c
	gcc -g -O2 -Wall `pkg-config glib-2.0 --cflags` -o $@ $^ `pkg-config glib-2.0 --libs`  `pcre-config --libs` -lglibivy


fms_steps_attitude: fms_steps_attitude.c
	gcc -g -O2 -Wall `pkg-config glib-2.0 --cflags` -o $@ $^ `pkg-config glib-2.0 --libs`  `pcre-config --libs` -lglibivy



clean:
	rm -f *~ core *.o *.bak .depend fdm_step fms_steps_attitude

