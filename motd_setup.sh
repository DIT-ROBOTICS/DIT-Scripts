#!/bin/bash

FileName='99-dit-news'
PATH='/etc/update-motd.d/'
echo "Copying motd config to $PATH"
cp ~/$FileName $PATH

