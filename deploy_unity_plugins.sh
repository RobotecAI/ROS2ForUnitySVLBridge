#!/usr/bin/env bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

ASSETS_DIR=$SCRIPTPATH/../../../

unzip "$1" 'Plugins/*' -d $ASSETS_DIR/
