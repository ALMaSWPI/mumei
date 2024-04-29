FROM wpimumei/base-images:latest

ARG TARGETPLATFORM

COPY . /mumei
RUN chmod u+x /mumei/tools/build_mumei.sh


RUN case ${TARGETPLATFORM} in \
         "linux/amd64")  TOOLCHAIN_PREFIX=x86_64  ;; \
         "linux/arm64")  TOOLCHAIN_PREFIX=arm64  ;; \
         "linux/arm/v7") TOOLCHAIN_PREFIX=armv7l  ;; \
         "linux/arm/v6") TOOLCHAIN_PREFIX=armel  ;; \
    esac \
    && /bin/bash /mumei/tools/build_mumei.sh "${TOOLCHAIN_PREFIX}"

