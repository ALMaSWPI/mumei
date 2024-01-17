FROM --platform=$BUILDPLATFORM wpihuron/base-images:latest AS build

ARG TARGETPLATFORM

COPY . /huron
RUN chmod u+x /huron/tools/build_huron.sh


RUN case ${TARGETPLATFORM} in \
         "linux/amd64")  TOOLCHAIN_PREFIX=x86_64  ;; \
         "linux/arm64")  TOOLCHAIN_PREFIX=arm64  ;; \
         "linux/arm/v7") TOOLCHAIN_PREFIX=armhf  ;; \
         "linux/arm/v6") TOOLCHAIN_PREFIX=armel  ;; \
         "linux/386")    TOOLCHAIN_PREFIX=i386   ;; \
    esac
RUN /huron/tools/build_huron.sh "${TOOLCHAIN_PREFIX}"

