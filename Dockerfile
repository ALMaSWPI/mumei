FROM wpihuron/base-images:latest

ARG TARGETPLATFORM

COPY . /huron
RUN chmod u+x /huron/tools/build_huron.sh


RUN case ${TARGETPLATFORM} in \
         "linux/amd64")  TOOLCHAIN_PREFIX=x86_64  ;; \
         "linux/arm64")  TOOLCHAIN_PREFIX=arm64  ;; \
         "linux/arm/v7") TOOLCHAIN_PREFIX=armv7l  ;; \
         "linux/arm/v6") TOOLCHAIN_PREFIX=armel  ;; \
    esac \
    && /bin/bash /huron/tools/build_huron.sh "${TOOLCHAIN_PREFIX}"

