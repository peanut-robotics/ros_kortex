#!/bin/bash

download_api() {
# download from PEANUT ROBOTICS' GOOGLE DRIVE FOLDER
wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=1YyGH-rXBljg7PcW-OQ0DK7YodBBblLwv' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=1YyGH-rXBljg7PcW-OQ0DK7YodBBblLwv" -O kortex_api-1.1.6.zip && rm -rf /tmp/cookies.txt
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while fetching the kortex api. code = ${RESULT}"
    exit $?
fi

rm ./cookies.txt
unzip -d kortex_api kortex_api-1.1.6.zip
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while extracting the kortex api. code = ${RESULT}"
    exit $?
fi
}

download_api
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while extracting the kortex api. code = ${RESULT}"
    rm -rf kortex_api*
    download_api
    if [ "${RESULT}" -ne 0 ]; then
      echo "ERROR while extracting the kortex api. code = ${RESULT}"
      exit $?
    fi
fi

cp -R kortex_api/cpp/linux_gcc_x86-64/include/ src/ros_kortex/kortex_api/
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while copying the kortex api header files. code = ${RESULT}"
    exit $?
fi

cp -R kortex_api/cpp/linux_gcc_x86-64/lib/ src/ros_kortex/kortex_api/
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while copying the kortex api library. code = ${RESULT}"
    exit $?
fi

chmod +x src/ros_kortex/kortex_api/lib/release/libKortexApi.a
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while executing chmod +x on the kortex api release library. code = ${RESULT}"
    exit $?
fi

chmod +x src/ros_kortex/kortex_api/lib/debug/libKortexApi.a
RESULT=$?
if [ "${RESULT}" -ne 0 ]; then
    echo "ERROR while executing chmod +x on the kortex api debug library. code = ${RESULT}"
    exit $?
fi

rm -rf kortex_api/ kortex_api-1.1.6.zip

exit ${RESULT}
