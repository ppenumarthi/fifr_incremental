#!/bin/bash
for i in $( ls topos ); do
  echo "./validate <topos/$i >$i.output.txt >>$i.error.txt"
  ./validate <topos/$i >$i.output.txt >>$i.error.txt
done
