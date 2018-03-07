#!/bin/bash
for i in $( ls topos ); do
   echo "./validate -f topos/$i -p 0 -v 1 -n 0 -r 1>$i.output.txt >>$i.error.txt"
  ./validate -f topos/$i -p 0 -v 1 -n 0 -r 1 >$i.output.txt >>$i.error.txt
done
