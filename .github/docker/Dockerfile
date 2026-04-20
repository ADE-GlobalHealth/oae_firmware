ARG IMAGE=ubuntu:24.04

# Use base image as extractor for the zip file
FROM ${IMAGE} AS base

RUN apt-get -y update && \
    apt-get -y install zip

COPY st-stm32cubeide_2.0.0_26820_20251114_1348_amd64.deb_bundle.sh.zip /tmp/stm32cubeide-installer.sh.zip

# Unzip &  delete zip file
RUN unzip -p /tmp/stm32cubeide-installer.sh.zip > /tmp/stm32cubeide-installer.sh && rm /tmp/stm32cubeide-installer.sh.zip

# ENV VARs
FROM ${IMAGE}
ENV STM32CUBEIDE_VERSION=1.19.0
ENV DEBIAN_FRONTEND=noninteractive
ENV LICENSE_ALREADY_ACCEPTED=1
ENV PATH="${PATH}:/opt/st/stm32cubeide_${STM32CUBEIDE_VERSION}"


# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    openjdk-17-jre \
    gcc g++ \
    libusb-1.0-0 \
    udev \
    unzip \
    && rm -rf /var/lib/apt/lists/*

COPY --from=base /tmp/stm32cubeide-installer.sh /tmp

# Install IDE and delete installer
RUN chmod +x /tmp/stm32cubeide-installer.sh && \
    /tmp/stm32cubeide-installer.sh --nox11 && \
    rm /tmp/stm32cubeide-installer.sh 