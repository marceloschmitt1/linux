# GSoC 2019 IIO Driver: AD7292

## Project

The Linux Foundation - Analog Devices AD7292 device driver project

## Organization

[The Linux Foundation](https://www.linuxfoundation.org/)

## Mentors

- Dragos Bogdan
- Stefan Popa
- Alexandru Ardelean

## Student

Marcelo Schmitt

## Code

[Get the code](https://github.com/marceloschmitt1/linux/tree/GSOC_2019_ad7292)

## Project Summary

This project's main goal is to develop a functional AD7292 driver that may be
maintained in the Linux mainline tree.  This repository is the result of the
2019 Google Summer of Code project "Analog Devices AD7292 device driver" by
Marcelo Schmitt. It is a fork of the Analog Devices Linux repository which
already contains most of the current implementation of the AD7292 driver. 

Further details on the work done can be seen on the following listed pull
requests and submitted patches.

### Pull requests made

- [AD7292](https://github.com/analogdevicesinc/linux/pull/436)
- [Add analog input channels and suport for voltage
regulator](https://github.com/analogdevicesinc/linux/pull/456)
- [Add support for reading single conversion
results](https://github.com/analogdevicesinc/linux/pull/529)

### Submitted patches

- [iio: adc: ad7292: add driver support for
AD7292](https://github.com/analogdevicesinc/linux/pull/436/commits/3d2c7522c01b19c19c77adfac0ab1546b56426bc)
- [dt-bindings: iio: adc: add DT docs for
AD7292](https://github.com/analogdevicesinc/linux/pull/436/commits/9bcafa90d1b01c36619b20f788fb769e1d43a727)
- [iio: adc: ad7292: add dtoverlay for
AD7292](https://github.com/analogdevicesinc/linux/pull/436/commits/774e39b7ac4bbcfb4104778e5e08e73b863923be)
- [iio: adc: ad7292: add IIO ADC
channels](https://github.com/analogdevicesinc/linux/pull/456/commits/be0aa43f1bcbf934402002693e496cd61bb15541)
- [iio: adc: ad7292: add voltage regulator
support](https://github.com/analogdevicesinc/linux/pull/456/commits/09434bb92f8828802ef7ec4e055e22d20518537b)
- [iio: adc: ad7292: add SPI reg read
functions](https://github.com/analogdevicesinc/linux/pull/456/commits/49db8a5d1421636f7e893a0836c4e6f4391c84b1)
- [iio: adc: ad7292: read single conversion
results](https://github.com/analogdevicesinc/linux/pull/529/commits/9f3cc26a3fb5d8331396debfd6f55803e92e887f)
- [iio: adc: ad7292: add support to read ADC
scale](https://github.com/analogdevicesinc/linux/pull/529/commits/f7b5cbeba36ad42143994a7c7862adad70c45006)

### Additional work for the community

In addition to contributing to The Linux Foundation through the AD7292 project,
I also encouraged developers to contribute to free software. As a member of the
[FLUSP](https://flusp.ime.usp.br/) students group, I helped to introduce people
to kernel development sharing experience and reviewing patches for our local
community. Even more, I also helped in organizing events to spread FLOSS
principles as well as development know-how. To say:

#### KernelDevDay (KDD)

[KernelDevDay](https://flusp.ime.usp.br/events/2019/04/01/kerneldevday_en/) was
a coding event organized by FLUSP (FLOSS@USP - IME-USP Extension Group) in
which participating teams are going to spend approximately 10 hours
contributing to some Linux kernel subsystem. In this first edition of the
event, we aim to contribute to some drivers from the sensors subsystem
(Industrial Input/Output - IIO) that are currently not ready to be merged into
the Linux Kernel main tree due to technical issues.

A summary of our accomplishments with KDD can be seen in the [KernelDevDay
Results](https://flusp.ime.usp.br/events/2019/06/14/kerneldevday-results_en/)
post at our site.

#### DebConf

We went together to [DebConf19](https://debconf19.debconf.org/) to present a
[talk](https://debconf19.debconf.org/talks/123-beneficios-de-uma-comunidade-local-de-contribuidores-floss/)
about our student's group.

#### Linuxdev-br

We were partners of [The Linux Developer Conference
Brazil](https://linuxdev-br.net/), an event that gathered Linux developers from
all around Brazil and other countries. The first [day of
workshops](https://cfp.linuxdev-br.net/2019/schedule/#2019-08-02) was hosted in
the [CCSL](http://ccsl.ime.usp.br/) FLOSS Competence Center at the [University
of São Paulo](https://www5.usp.br/). Home for the FLUSP group and crib for many
FLOSS developers.

#### Incomming tutorial posts

As a complement to my code contributions, I am working on a series of tutorial
posts to provide a guide to introduce developers to IIO driver development.
These tutorials shall be available on my
[website](https://linux.ime.usp.br/~marcelosc/) and also at
[FLUSP's](https://flusp.ime.usp.br/).

### What's left to do

- current ADC readings are not garanteed to be corrent since the BUSY pin signal
is not being monitored yet.
- add support for internal temperature readings
- add support for operating the DACs
- add support for customizing configurations at the register bank
- add support for alarm and custom GPIO features
- implement condinuous readings within a buffer triggered mode

### Acknowledgements

Special thanks to Dragos Bogdan, Stefan Popa, Alexandru Ardelean (GSoC
mentors), for providing me guidance from long before the GSoC program has
started; Rodrigo Siqueira Jordão for teaching me the basics of kernel
development and motivating me to go further; Paulo R. M. Meirelles for being
understanding and supportive during this project; everyone in our local FLOSS
community, this GSoC project wouldn't be so meaningful without you. Thank you
all!
