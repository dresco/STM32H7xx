# درایور grblHAL STM32H7xx — فورک سازگاری BTT Scylla

[English](../README.md) | [فارسی](README.fa.md)

این مخزن یک Fork عمومی از پروژه [`dresco/STM32H7xx`](https://github.com/dresco/STM32H7xx) است که تمرکز اصلی آن روی **BTT Scylla V1 با میکروکنترلر STM32H723** و سازگار نگه‌داشتن آن با Core جدید grblHAL و نسخه‌های جدید Submoduleها است.

هدف مرحله فعلی پروژه ایجاد یک مبنای **تمیز، قابل بازتولید، مستند و قابل Build** برای Scylla V1 است؛ قبل از اینکه تنظیمات اختصاصی ماشین CNC به آن اضافه شود.

> **وضعیت فعلی اعتبارسنجی:** Build کامل و Clean Build با PlatformIO با موفقیت انجام شده است. تست کامل روی سخت‌افزار واقعی هنوز انجام نشده و پس از تست برد ثبت خواهد شد.

---

## چرا این Fork ایجاد شد؟

نسخه فعلی Driver مربوط به STM32H7xx نسبت به API جدید Core پروژه grblHAL عقب‌تر بود.

در Core جدید grblHAL، callback قدیمی زیر حذف شده است:

```c
hal.settings_changed
```

و مدیریت تغییر تنظیمات به event جدید زیر منتقل شده است:

```c
grbl.on_settings_changed
```

بعد از اینکه Core و Plugin/Submoduleهای پروژه به نسخه‌های جدید upstream به‌روزرسانی شدند، Build مربوط به BTT Scylla با خطا متوقف شد، چون Driver STM32H7xx هنوز از API حذف‌شده استفاده می‌کرد.

در این Fork، Driver با API جدید هماهنگ شد و سپس Build کامل Scylla از ابتدا با موفقیت انجام شد.

---

## تغییر سازگاری انجام‌شده

فایل‌های زیر اصلاح شدند:

- `Src/driver.c`
- `Src/driver_spindles.c`
- `Src/encoders.c`
- `Src/neopixel_spi.c`

تمام استفاده‌های مرتبط از:

```c
hal.settings_changed
```

به:

```c
grbl.on_settings_changed
```

تغییر داده شدند.

این تغییر با مدل event جدید grblHAL Core هماهنگ است.

---

## وضعیت فعلی پروژه

| مورد | وضعیت |
|---|---|
| هدف BTT Scylla V1 / STM32H723 | Build تأیید شده |
| محیط PlatformIO | تأیید شده |
| USB CDC | در Build فعال |
| SD Card | در Build فعال |
| TMC5160 | در Build فعال |
| Clean Build کامل از صفر | تأیید شده |
| تست Runtime روی برد واقعی | در انتظار تست |
| تنظیمات اختصاصی CNC | مرحله بعد |

Environment تأییدشده PlatformIO:

```text
btt_scylla_h723_tmc5160
```

این Environment نسخه **بدون Bootloader** است و برای فلش مستقیم Firmware در نظر گرفته شده است.

Environment زیر مربوط به Build دارای Bootloader جداگانه است و مبنای فعلی ما نیست:

```text
btt_scylla_h723_tmc5160_bl128
```

---

## مبنای نسخه‌های پروژه

در زمان ساخت این Baseline:

| بخش | Revision |
|---|---|
| مبنای upstream پروژه STM32H7xx | `bfe5d4f6ec271cc9929336244c3b1c8e55d1d040` |
| grblHAL Core | `516e5ad80757bd2eba86bff18feb613ca121dc16` |
| Commit آپدیت Submoduleها | `3279baa` |
| Commit سازگاری Driver با Core جدید | `15dd0e3` |
| Commit تنظیم PlatformIO / VS Code | `7eeb637` |

پروژه ابتدا از آخرین نسخه موجود Driver STM32H7xx ساخته شد، سپس Submoduleهای تعریف‌شده در پروژه به Revisionهای جدیدشان به‌روزرسانی شدند و پس از آن Build انجام شد.

> نکته: سرعت توسعه Core پروژه grblHAL ممکن است از بعضی Driverهای MCU بیشتر باشد. به همین دلیل در این Fork Revision دقیق نسخه‌ای که Build موفق با آن انجام شده ثبت می‌شود.

---

## محیط Build تأییدشده

Clean Build با مشخصات زیر انجام شده است:

```text
PlatformIO Core: 6.2.0
Platform: ST STM32 20.0.0
Framework: STM32CubeH7 1.12.1
Target MCU: STM32H723VGT6
PlatformIO environment: btt_scylla_h723_tmc5160
```

مصرف حافظه در Build موفق:

```text
RAM:   31,484 / 577,536 bytes     (5.5%)
Flash: 321,700 / 1,048,576 bytes  (30.7%)
```

---

## بردهای پشتیبانی‌شده از upstream

Driver اصلی STM32H7xx از چندین برد مبتنی بر STM32H7 پشتیبانی می‌کند، از جمله:

- Nucleo-H743ZI
- Nucleo-H723ZG
- WeAct MiniSTM32H7xx
- BTT SKR 3 EZ
- BTT Octopus Max EZ
- BTT Scylla
- سایر Targetهای موجود در upstream

تمرکز این Fork در مرحله فعلی روی **BTT Scylla V1** است.

---

## پیش‌نیازها

در ویندوز، محیطی که با آن Build تأیید شد شامل موارد زیر بود:

- Git
- Python
- Visual Studio Code — اختیاری ولی پیشنهادشده
- افزونه PlatformIO IDE در VS Code
- PlatformIO Core CLI

VS Code برای Build اجباری نیست و پروژه می‌تواند فقط با خط فرمان PlatformIO نیز Build شود.

---

## Clone کردن پروژه

چون پروژه از Git Submodule استفاده می‌کند، Clone بهتر است به‌صورت recursive انجام شود:

```bash
git clone --recursive https://github.com/coolboysalam/STM32H7xx.git
cd STM32H7xx
git checkout scylla-grblhal-latest
```

اگر پروژه بدون `--recursive` Clone شده باشد:

```bash
git submodule update --init --recursive
```

را اجرا کنید.

---

## Build کردن Firmware برای BTT Scylla V1

ابتدا Build قبلی را پاک کنید:

```bash
pio run -e btt_scylla_h723_tmc5160 -t clean
```

سپس Firmware را Build کنید:

```bash
pio run -e btt_scylla_h723_tmc5160
```

در صورت موفقیت، خروجی باید در انتها شامل:

```text
SUCCESS
```

باشد.

فایل‌های اصلی خروجی در این مسیر ساخته می‌شوند:

```text
.pio/build/btt_scylla_h723_tmc5160/firmware.bin
.pio/build/btt_scylla_h723_tmc5160/firmware.elf
```

---

## نکته مهم Bootloader

برای Scylla چند Environment مختلف وجود دارد.

Baseline فعلی این پروژه از:

```text
btt_scylla_h723_tmc5160
```

استفاده می‌کند.

و از:

```text
btt_scylla_h723_tmc5160_bl128
```

استفاده نمی‌کند.

نسخه Bootloader فقط زمانی باید استفاده شود که Bootloader متناظر واقعاً روی برد نصب باشد و Layout حافظه نیز مطابق آن تنظیم شده باشد.

---

## مستندات برنامه‌ریزی‌شده برای Scylla

در مراحل بعد فایل‌های مستندات تخصصی زیر اضافه خواهند شد:

- Pin Map کامل Scylla V1
- تنظیم Limit و Homing
- ورودی‌های Reset / Feed Hold / Cycle Start
- Probe و Toolsetter
- خروجی‌های AUX و مخصوصاً AUX10
- تنظیمات TMC5160
- کنترل Spindle
- Modbus / VFD
- منطق E-Stop و Safety
- روش فلش مستقیم Firmware
- چک‌لیست کامل تست سخت‌افزار

این مستندات در پوشه `docs/` نگهداری خواهند شد.

---

## ساختار Remoteهای Git

ساختار پیشنهادی پروژه:

```text
origin   -> https://github.com/coolboysalam/STM32H7xx.git
upstream -> https://github.com/dresco/STM32H7xx.git
```

برای دریافت تغییرات جدید upstream:

```bash
git fetch upstream
```

برای هماهنگ‌سازی Submoduleها:

```bash
git submodule sync --recursive
git submodule update --remote --recursive
```

بعد از هر آپدیت Core یا Submoduleها حتماً Clean Build انجام شود، چون APIهای Core ممکن است سریع‌تر از Driverهای MCU تغییر کنند.

---

## سیاست توسعه این Fork

روش توسعه این پروژه به این صورت خواهد بود:

1. همیشه از یک Baseline با Build موفق شروع شود.
2. آپدیت upstream آگاهانه و کنترل‌شده انجام شود.
3. Compatibility Patchها در Commitهای جدا و واضح نگهداری شوند.
4. پس از هر تغییر سازگاری، Clean Build انجام شود.
5. قبل از اعلام «تست‌شده»، تست سخت‌افزار واقعی انجام شود.
6. تمام تنظیمات و تغییرات مهم مستند شوند.

هدف این است که Firmware قابل بازتولید باشد و در صورت بروز Regression بتوان علت آن را سریع پیدا کرد.

---

## پروژه‌های upstream و اعتبار

این مخزن بر پایه کار توسعه‌دهندگان اصلی grblHAL و نگهدارندگان Driver STM32H7xx ساخته شده است.

پروژه‌های اصلی:

- [grblHAL Core](https://github.com/grblHAL/core)
- [STM32H7xx driver by dresco](https://github.com/dresco/STM32H7xx)

تمام اعتبار، تاریخچه و Copyright اصلی باید حفظ شود.

---

## مجوز

این Fork مجوزها و Copyrightهای اصلی upstream را حفظ می‌کند.

grblHAL و Driver STM32H7xx تحت شرایط GNU General Public License موجود در همین repository منتشر می‌شوند. برای جزئیات فایل `COPYING` و License Header فایل‌های سورس را ببینید.

---

## مرحله فعلی پروژه

**مرحله ۱ — ساخت Baseline تمیز و سازگار با Core جدید:** تکمیل شده.

**مرحله ۲ — تنظیم و اعتبارسنجی سخت‌افزار BTT Scylla V1 برای CNC:** در حال انجام.

نتیجه تست Firmware روی برد واقعی پس از انجام تست به همین مستندات اضافه خواهد شد.
