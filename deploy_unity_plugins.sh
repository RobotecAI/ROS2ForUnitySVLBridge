#!/usr/bin/env bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

ASSETS_DIR=$SCRIPTPATH/../../../

unzip $SCRIPTPATH/plugins.zip 'Plugins/*' -d $ASSETS_DIR/
