#!/bin/bash

cd "$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

echo ""
echo "Downloading simulation environments..."
echo ""

ggID='13RRRXyip7n2jP4DrWhTfusG1MKvMtpH3'
ggURL='https://drive.google.com/uc?export=download'

filename="$(curl -sc /tmp/gcokie "${ggURL}&id=${ggID}" | grep -o '="uc-name.*</span>' | sed 's/.*">//;s/<.a> .*//')"
getcode="$(awk '/_warning_/ {print $NF}' /tmp/gcokie)"

curl -Lb /tmp/gcokie "${ggURL}&confirm=${getcode}&id=${ggID}" -o "${filename}"

echo ""
echo "Unzipping files..."
echo ""

unzip "${filename}"

rm "${filename}"

echo ""
echo "Done, simulation environments are kept in 'src/nctu_simulation_env/mesh'."
echo ""
