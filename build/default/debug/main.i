# 1 "main.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 288 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/language_support.h" 1 3
# 2 "<built-in>" 2
# 1 "main.c" 2
# 25 "main.c"
#pragma config OSC = XT
#pragma config OSCS = OFF


#pragma config PWRT = OFF
#pragma config BOR = OFF
#pragma config BORV = 20


#pragma config WDT = OFF
#pragma config WDTPS = 128


#pragma config CCP2MUX = OFF


#pragma config STVR = OFF
#pragma config LVP = OFF


#pragma config CP0 = OFF
#pragma config CP1 = OFF
#pragma config CP2 = OFF
#pragma config CP3 = OFF


#pragma config CPB = OFF
#pragma config CPD = OFF


#pragma config WRT0 = OFF
#pragma config WRT1 = OFF
#pragma config WRT2 = OFF
#pragma config WRT3 = OFF


#pragma config WRTC = OFF
#pragma config WRTB = OFF
#pragma config WRTD = OFF


#pragma config EBTR0 = OFF
#pragma config EBTR1 = OFF
#pragma config EBTR2 = OFF
#pragma config EBTR3 = OFF


#pragma config EBTRB = OFF

# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/stdio.h" 1 3



# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/musl_xc8.h" 1 3
# 5 "/opt/microchip/xc8/v2.32/pic/include/c99/stdio.h" 2 3





# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/features.h" 1 3
# 11 "/opt/microchip/xc8/v2.32/pic/include/c99/stdio.h" 2 3
# 24 "/opt/microchip/xc8/v2.32/pic/include/c99/stdio.h" 3
# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 1 3





typedef void * va_list[1];




typedef void * __isoc_va_list[1];
# 122 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef unsigned size_t;
# 137 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef long ssize_t;
# 168 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef __int24 int24_t;
# 204 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef __uint24 uint24_t;
# 246 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef long long off_t;
# 399 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef struct _IO_FILE FILE;
# 25 "/opt/microchip/xc8/v2.32/pic/include/c99/stdio.h" 2 3
# 52 "/opt/microchip/xc8/v2.32/pic/include/c99/stdio.h" 3
typedef union _G_fpos64_t {
 char __opaque[16];
 double __align;
} fpos_t;

extern FILE *const stdin;
extern FILE *const stdout;
extern FILE *const stderr;





FILE *fopen(const char *restrict, const char *restrict);
FILE *freopen(const char *restrict, const char *restrict, FILE *restrict);
int fclose(FILE *);

int remove(const char *);
int rename(const char *, const char *);

int feof(FILE *);
int ferror(FILE *);
int fflush(FILE *);
void clearerr(FILE *);

int fseek(FILE *, long, int);
long ftell(FILE *);
void rewind(FILE *);

int fgetpos(FILE *restrict, fpos_t *restrict);
int fsetpos(FILE *, const fpos_t *);

size_t fread(void *restrict, size_t, size_t, FILE *restrict);
size_t fwrite(const void *restrict, size_t, size_t, FILE *restrict);

int fgetc(FILE *);
int getc(FILE *);
int getchar(void);
int ungetc(int, FILE *);

int fputc(int, FILE *);
int putc(int, FILE *);
int putchar(int);

char *fgets(char *restrict, int, FILE *restrict);

char *gets(char *);


int fputs(const char *restrict, FILE *restrict);
int puts(const char *);

#pragma printf_check(printf) const
#pragma printf_check(vprintf) const
#pragma printf_check(sprintf) const
#pragma printf_check(snprintf) const
#pragma printf_check(vsprintf) const
#pragma printf_check(vsnprintf) const

int printf(const char *restrict, ...);
int fprintf(FILE *restrict, const char *restrict, ...);
int sprintf(char *restrict, const char *restrict, ...);
int snprintf(char *restrict, size_t, const char *restrict, ...);

int vprintf(const char *restrict, __isoc_va_list);
int vfprintf(FILE *restrict, const char *restrict, __isoc_va_list);
int vsprintf(char *restrict, const char *restrict, __isoc_va_list);
int vsnprintf(char *restrict, size_t, const char *restrict, __isoc_va_list);

int scanf(const char *restrict, ...);
int fscanf(FILE *restrict, const char *restrict, ...);
int sscanf(const char *restrict, const char *restrict, ...);
int vscanf(const char *restrict, __isoc_va_list);
int vfscanf(FILE *restrict, const char *restrict, __isoc_va_list);
int vsscanf(const char *restrict, const char *restrict, __isoc_va_list);

void perror(const char *);

int setvbuf(FILE *restrict, char *restrict, int, size_t);
void setbuf(FILE *restrict, char *restrict);

char *tmpnam(char *);
FILE *tmpfile(void);




FILE *fmemopen(void *restrict, size_t, const char *restrict);
FILE *open_memstream(char **, size_t *);
FILE *fdopen(int, const char *);
FILE *popen(const char *, const char *);
int pclose(FILE *);
int fileno(FILE *);
int fseeko(FILE *, off_t, int);
off_t ftello(FILE *);
int dprintf(int, const char *restrict, ...);
int vdprintf(int, const char *restrict, __isoc_va_list);
void flockfile(FILE *);
int ftrylockfile(FILE *);
void funlockfile(FILE *);
int getc_unlocked(FILE *);
int getchar_unlocked(void);
int putc_unlocked(int, FILE *);
int putchar_unlocked(int);
ssize_t getdelim(char **restrict, size_t *restrict, int, FILE *restrict);
ssize_t getline(char **restrict, size_t *restrict, FILE *restrict);
int renameat(int, const char *, int, const char *);
char *ctermid(char *);







char *tempnam(const char *, const char *);
# 73 "main.c" 2

# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/stdint.h" 1 3
# 22 "/opt/microchip/xc8/v2.32/pic/include/c99/stdint.h" 3
# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 1 3
# 127 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef unsigned long uintptr_t;
# 142 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef long intptr_t;
# 158 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef signed char int8_t;




typedef short int16_t;
# 173 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef long int32_t;





typedef long long int64_t;
# 188 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef long long intmax_t;





typedef unsigned char uint8_t;




typedef unsigned short uint16_t;
# 209 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef unsigned long uint32_t;





typedef unsigned long long uint64_t;
# 229 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef unsigned long long uintmax_t;
# 23 "/opt/microchip/xc8/v2.32/pic/include/c99/stdint.h" 2 3

typedef int8_t int_fast8_t;

typedef int64_t int_fast64_t;


typedef int8_t int_least8_t;
typedef int16_t int_least16_t;

typedef int24_t int_least24_t;
typedef int24_t int_fast24_t;

typedef int32_t int_least32_t;

typedef int64_t int_least64_t;


typedef uint8_t uint_fast8_t;

typedef uint64_t uint_fast64_t;


typedef uint8_t uint_least8_t;
typedef uint16_t uint_least16_t;

typedef uint24_t uint_least24_t;
typedef uint24_t uint_fast24_t;

typedef uint32_t uint_least32_t;

typedef uint64_t uint_least64_t;
# 144 "/opt/microchip/xc8/v2.32/pic/include/c99/stdint.h" 3
# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/stdint.h" 1 3
typedef int16_t int_fast16_t;
typedef int32_t int_fast32_t;
typedef uint16_t uint_fast16_t;
typedef uint32_t uint_fast32_t;
# 145 "/opt/microchip/xc8/v2.32/pic/include/c99/stdint.h" 2 3
# 74 "main.c" 2

# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/string.h" 1 3
# 25 "/opt/microchip/xc8/v2.32/pic/include/c99/string.h" 3
# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 1 3
# 411 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef struct __locale_struct * locale_t;
# 26 "/opt/microchip/xc8/v2.32/pic/include/c99/string.h" 2 3

void *memcpy (void *restrict, const void *restrict, size_t);
void *memmove (void *, const void *, size_t);
void *memset (void *, int, size_t);
int memcmp (const void *, const void *, size_t);
void *memchr (const void *, int, size_t);

char *strcpy (char *restrict, const char *restrict);
char *strncpy (char *restrict, const char *restrict, size_t);

char *strcat (char *restrict, const char *restrict);
char *strncat (char *restrict, const char *restrict, size_t);

int strcmp (const char *, const char *);
int strncmp (const char *, const char *, size_t);

int strcoll (const char *, const char *);
size_t strxfrm (char *restrict, const char *restrict, size_t);

char *strchr (const char *, int);
char *strrchr (const char *, int);

size_t strcspn (const char *, const char *);
size_t strspn (const char *, const char *);
char *strpbrk (const char *, const char *);
char *strstr (const char *, const char *);
char *strtok (char *restrict, const char *restrict);

size_t strlen (const char *);

char *strerror (int);
# 65 "/opt/microchip/xc8/v2.32/pic/include/c99/string.h" 3
char *strtok_r (char *restrict, const char *restrict, char **restrict);
int strerror_r (int, char *, size_t);
char *stpcpy(char *restrict, const char *restrict);
char *stpncpy(char *restrict, const char *restrict, size_t);
size_t strnlen (const char *, size_t);
char *strdup (const char *);
char *strndup (const char *, size_t);
char *strsignal(int);
char *strerror_l (int, locale_t);
int strcoll_l (const char *, const char *, locale_t);
size_t strxfrm_l (char *restrict, const char *restrict, size_t, locale_t);




void *memccpy (void *restrict, const void *restrict, int, size_t);
# 75 "main.c" 2

# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/stdbool.h" 1 3
# 76 "main.c" 2

# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/math.h" 1 3
# 15 "/opt/microchip/xc8/v2.32/pic/include/c99/math.h" 3
# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 1 3
# 33 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef float float_t;




typedef double double_t;
# 16 "/opt/microchip/xc8/v2.32/pic/include/c99/math.h" 2 3
# 42 "/opt/microchip/xc8/v2.32/pic/include/c99/math.h" 3
int __fpclassifyf(float);







int __signbitf(float);
# 59 "/opt/microchip/xc8/v2.32/pic/include/c99/math.h" 3
double acos(double);
float acosf(float);
long double acosl(long double);



double acosh(double);
float acoshf(float);
long double acoshl(long double);



double asin(double);
float asinf(float);
long double asinl(long double);



double asinh(double);
float asinhf(float);
long double asinhl(long double);



double atan(double);
float atanf(float);
long double atanl(long double);



double atan2(double, double);
float atan2f(float, float);
long double atan2l(long double, long double);



double atanh(double);
float atanhf(float);
long double atanhl(long double);



double cbrt(double);
float cbrtf(float);
long double cbrtl(long double);



double ceil(double);
float ceilf(float);
long double ceill(long double);



double copysign(double, double);
float copysignf(float, float);
long double copysignl(long double, long double);



double cos(double);
float cosf(float);
long double cosl(long double);



double cosh(double);
float coshf(float);
long double coshl(long double);



double erf(double);
float erff(float);
long double erfl(long double);



double erfc(double);
float erfcf(float);
long double erfcl(long double);



double exp(double);
float expf(float);
long double expl(long double);



double exp2(double);
float exp2f(float);
long double exp2l(long double);



double expm1(double);
float expm1f(float);
long double expm1l(long double);



double fabs(double);
float fabsf(float);
long double fabsl(long double);



double fdim(double, double);
float fdimf(float, float);
long double fdiml(long double, long double);



double floor(double);
float floorf(float);
long double floorl(long double);



double fma(double, double, double);
float fmaf(float, float, float);
long double fmal(long double, long double, long double);



double fmax(double, double);
float fmaxf(float, float);
long double fmaxl(long double, long double);



double fmin(double, double);
float fminf(float, float);
long double fminl(long double, long double);



double fmod(double, double);
float fmodf(float, float);
long double fmodl(long double, long double);



double frexp(double, int *);
float frexpf(float, int *);
long double frexpl(long double, int *);



double hypot(double, double);
float hypotf(float, float);
long double hypotl(long double, long double);



int ilogb(double);
int ilogbf(float);
int ilogbl(long double);



double ldexp(double, int);
float ldexpf(float, int);
long double ldexpl(long double, int);



double lgamma(double);
float lgammaf(float);
long double lgammal(long double);



long long llrint(double);
long long llrintf(float);
long long llrintl(long double);



long long llround(double);
long long llroundf(float);
long long llroundl(long double);



double log(double);
float logf(float);
long double logl(long double);



double log10(double);
float log10f(float);
long double log10l(long double);



double log1p(double);
float log1pf(float);
long double log1pl(long double);



double log2(double);
float log2f(float);
long double log2l(long double);



double logb(double);
float logbf(float);
long double logbl(long double);



long lrint(double);
long lrintf(float);
long lrintl(long double);



long lround(double);
long lroundf(float);
long lroundl(long double);



double modf(double, double *);
float modff(float, float *);
long double modfl(long double, long double *);



double nan(const char *);
float nanf(const char *);
long double nanl(const char *);



double nearbyint(double);
float nearbyintf(float);
long double nearbyintl(long double);



double nextafter(double, double);
float nextafterf(float, float);
long double nextafterl(long double, long double);



double nexttoward(double, long double);
float nexttowardf(float, long double);
long double nexttowardl(long double, long double);




double pow(double, double);
__attribute__((nonreentrant)) float powf(float, float);
long double powl(long double, long double);



double remainder(double, double);
float remainderf(float, float);
long double remainderl(long double, long double);



double remquo(double, double, int *);
float remquof(float, float, int *);
long double remquol(long double, long double, int *);



double rint(double);
float rintf(float);
long double rintl(long double);



double round(double);
float roundf(float);
long double roundl(long double);



double scalbln(double, long);
float scalblnf(float, long);
long double scalblnl(long double, long);



double scalbn(double, int);
float scalbnf(float, int);
long double scalbnl(long double, int);



double sin(double);
float sinf(float);
long double sinl(long double);



double sinh(double);
float sinhf(float);
long double sinhl(long double);



double sqrt(double);
float sqrtf(float);
long double sqrtl(long double);



double tan(double);
float tanf(float);
long double tanl(long double);



double tanh(double);
float tanhf(float);
long double tanhl(long double);



double tgamma(double);
float tgammaf(float);
long double tgammal(long double);



double trunc(double);
float truncf(float);
long double truncl(long double);
# 423 "/opt/microchip/xc8/v2.32/pic/include/c99/math.h" 3
extern int signgam;

double j0(double);
double j1(double);
double jn(int, double);

double y0(double);
double y1(double);
double yn(int, double);
# 77 "main.c" 2

# 1 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/xc.h" 1 3
# 18 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/xc.h" 3
extern const char __xc8_OPTIM_SPEED;

extern double __fpnormalize(double);



# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/xc8debug.h" 1 3



# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/stdlib.h" 1 3
# 21 "/opt/microchip/xc8/v2.32/pic/include/c99/stdlib.h" 3
# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 1 3
# 18 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef long int wchar_t;
# 22 "/opt/microchip/xc8/v2.32/pic/include/c99/stdlib.h" 2 3

int atoi (const char *);
long atol (const char *);
long long atoll (const char *);
double atof (const char *);

float strtof (const char *restrict, char **restrict);
double strtod (const char *restrict, char **restrict);
long double strtold (const char *restrict, char **restrict);



long strtol (const char *restrict, char **restrict, int);
unsigned long strtoul (const char *restrict, char **restrict, int);
long long strtoll (const char *restrict, char **restrict, int);
unsigned long long strtoull (const char *restrict, char **restrict, int);

int rand (void);
void srand (unsigned);

          void abort (void);
int atexit (void (*) (void));
          void exit (int);
          void _Exit (int);

void *bsearch (const void *, const void *, size_t, size_t, int (*)(const void *, const void *));

__attribute__((nonreentrant)) void qsort (void *, size_t, size_t, int (*)(const void *, const void *));

int abs (int);
long labs (long);
long long llabs (long long);

typedef struct { int quot, rem; } div_t;
typedef struct { long quot, rem; } ldiv_t;
typedef struct { long long quot, rem; } lldiv_t;

div_t div (int, int);
ldiv_t ldiv (long, long);
lldiv_t lldiv (long long, long long);

typedef struct { unsigned int quot, rem; } udiv_t;
typedef struct { unsigned long quot, rem; } uldiv_t;
udiv_t udiv (unsigned int, unsigned int);
uldiv_t uldiv (unsigned long, unsigned long);





size_t __ctype_get_mb_cur_max(void);
# 5 "/opt/microchip/xc8/v2.32/pic/include/c99/xc8debug.h" 2 3







#pragma intrinsic(__builtin_software_breakpoint)
extern void __builtin_software_breakpoint(void);
# 24 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/xc.h" 2 3








# 1 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/pic18.h" 1 3




# 1 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/htc.h" 1 3



# 1 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/xc.h" 1 3
# 5 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/htc.h" 2 3
# 6 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/pic18.h" 2 3


# 1 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/pic18_chip_select.h" 1 3
# 314 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/pic18_chip_select.h" 3
# 1 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 1 3
# 44 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
# 1 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/__at.h" 1 3
# 45 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 2 3







extern volatile unsigned char PORTA __attribute__((address(0xF80)));

__asm("PORTA equ 0F80h");


typedef union {
    struct {
        unsigned RA0 :1;
        unsigned RA1 :1;
        unsigned RA2 :1;
        unsigned RA3 :1;
        unsigned RA4 :1;
        unsigned RA5 :1;
        unsigned RA6 :1;
    };
    struct {
        unsigned AN0 :1;
        unsigned AN1 :1;
        unsigned AN2 :1;
        unsigned AN3 :1;
        unsigned :1;
        unsigned AN4 :1;
        unsigned OSC2 :1;
    };
    struct {
        unsigned :2;
        unsigned VREFM :1;
        unsigned VREFP :1;
        unsigned T0CKI :1;
        unsigned SS :1;
        unsigned CLKO :1;
    };
    struct {
        unsigned :5;
        unsigned LVDIN :1;
    };
    struct {
        unsigned ULPWUIN :1;
    };
} PORTAbits_t;
extern volatile PORTAbits_t PORTAbits __attribute__((address(0xF80)));
# 197 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char PORTB __attribute__((address(0xF81)));

__asm("PORTB equ 0F81h");


typedef union {
    struct {
        unsigned RB0 :1;
        unsigned RB1 :1;
        unsigned RB2 :1;
        unsigned RB3 :1;
        unsigned RB4 :1;
        unsigned RB5 :1;
        unsigned RB6 :1;
        unsigned RB7 :1;
    };
    struct {
        unsigned INT0 :1;
        unsigned INT1 :1;
        unsigned INT2 :1;
        unsigned CCP2 :1;
        unsigned :1;
        unsigned PGM :1;
        unsigned PGC :1;
        unsigned PGD :1;
    };
    struct {
        unsigned :3;
        unsigned CCP2A :1;
    };
    struct {
        unsigned :3;
        unsigned CCP2_PA2 :1;
    };
} PORTBbits_t;
extern volatile PORTBbits_t PORTBbits __attribute__((address(0xF81)));
# 322 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char PORTC __attribute__((address(0xF82)));

__asm("PORTC equ 0F82h");


typedef union {
    struct {
        unsigned RC0 :1;
        unsigned RC1 :1;
        unsigned RC2 :1;
        unsigned RC3 :1;
        unsigned RC4 :1;
        unsigned RC5 :1;
        unsigned RC6 :1;
        unsigned RC7 :1;
    };
    struct {
        unsigned T1OSO :1;
        unsigned T1OSI :1;
        unsigned :1;
        unsigned SCK :1;
        unsigned SDI :1;
        unsigned SDO :1;
        unsigned TX :1;
        unsigned RX :1;
    };
    struct {
        unsigned T1CKI :1;
        unsigned CCP2 :1;
        unsigned CCP1 :1;
        unsigned SCL :1;
        unsigned SDA :1;
        unsigned :1;
        unsigned CK :1;
        unsigned DT :1;
    };
    struct {
        unsigned :1;
        unsigned PA2 :1;
        unsigned PA1 :1;
    };
} PORTCbits_t;
extern volatile PORTCbits_t PORTCbits __attribute__((address(0xF82)));
# 489 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char PORTD __attribute__((address(0xF83)));

__asm("PORTD equ 0F83h");


typedef union {
    struct {
        unsigned RD0 :1;
        unsigned RD1 :1;
        unsigned RD2 :1;
        unsigned RD3 :1;
        unsigned RD4 :1;
        unsigned RD5 :1;
        unsigned RD6 :1;
        unsigned RD7 :1;
    };
    struct {
        unsigned PSP0 :1;
        unsigned PSP1 :1;
        unsigned PSP2 :1;
        unsigned PSP3 :1;
        unsigned PSP4 :1;
        unsigned PSP5 :1;
        unsigned PSP6 :1;
        unsigned PSP7 :1;
    };
    struct {
        unsigned :7;
        unsigned SS2 :1;
    };
} PORTDbits_t;
extern volatile PORTDbits_t PORTDbits __attribute__((address(0xF83)));
# 610 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char PORTE __attribute__((address(0xF84)));

__asm("PORTE equ 0F84h");


typedef union {
    struct {
        unsigned RE0 :1;
        unsigned RE1 :1;
        unsigned RE2 :1;
    };
    struct {
        unsigned AN5 :1;
        unsigned AN6 :1;
        unsigned AN7 :1;
    };
    struct {
        unsigned RD :1;
        unsigned WR :1;
        unsigned CS :1;
    };
    struct {
        unsigned PD2 :1;
        unsigned PC2 :1;
        unsigned CCP10 :1;
    };
    struct {
        unsigned RDE :1;
        unsigned WRE :1;
        unsigned PB2 :1;
    };
} PORTEbits_t;
extern volatile PORTEbits_t PORTEbits __attribute__((address(0xF84)));
# 722 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char LATA __attribute__((address(0xF89)));

__asm("LATA equ 0F89h");


typedef union {
    struct {
        unsigned LATA0 :1;
        unsigned LATA1 :1;
        unsigned LATA2 :1;
        unsigned LATA3 :1;
        unsigned LATA4 :1;
        unsigned LATA5 :1;
        unsigned LATA6 :1;
    };
    struct {
        unsigned LA0 :1;
        unsigned LA1 :1;
        unsigned LA2 :1;
        unsigned LA3 :1;
        unsigned LA4 :1;
        unsigned LA5 :1;
        unsigned LA6 :1;
    };
} LATAbits_t;
extern volatile LATAbits_t LATAbits __attribute__((address(0xF89)));
# 822 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char LATB __attribute__((address(0xF8A)));

__asm("LATB equ 0F8Ah");


typedef union {
    struct {
        unsigned LATB0 :1;
        unsigned LATB1 :1;
        unsigned LATB2 :1;
        unsigned LATB3 :1;
        unsigned LATB4 :1;
        unsigned LATB5 :1;
        unsigned LATB6 :1;
        unsigned LATB7 :1;
    };
    struct {
        unsigned LB0 :1;
        unsigned LB1 :1;
        unsigned LB2 :1;
        unsigned LB3 :1;
        unsigned LB4 :1;
        unsigned LB5 :1;
        unsigned LB6 :1;
        unsigned LB7 :1;
    };
} LATBbits_t;
extern volatile LATBbits_t LATBbits __attribute__((address(0xF8A)));
# 934 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char LATC __attribute__((address(0xF8B)));

__asm("LATC equ 0F8Bh");


typedef union {
    struct {
        unsigned LATC0 :1;
        unsigned LATC1 :1;
        unsigned LATC2 :1;
        unsigned LATC3 :1;
        unsigned LATC4 :1;
        unsigned LATC5 :1;
        unsigned LATC6 :1;
        unsigned LATC7 :1;
    };
    struct {
        unsigned LC0 :1;
        unsigned LC1 :1;
        unsigned LC2 :1;
        unsigned LC3 :1;
        unsigned LC4 :1;
        unsigned LC5 :1;
        unsigned LC6 :1;
        unsigned LC7 :1;
    };
} LATCbits_t;
extern volatile LATCbits_t LATCbits __attribute__((address(0xF8B)));
# 1046 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char LATD __attribute__((address(0xF8C)));

__asm("LATD equ 0F8Ch");


typedef union {
    struct {
        unsigned LATD0 :1;
        unsigned LATD1 :1;
        unsigned LATD2 :1;
        unsigned LATD3 :1;
        unsigned LATD4 :1;
        unsigned LATD5 :1;
        unsigned LATD6 :1;
        unsigned LATD7 :1;
    };
    struct {
        unsigned LD0 :1;
        unsigned LD1 :1;
        unsigned LD2 :1;
        unsigned LD3 :1;
        unsigned LD4 :1;
        unsigned LD5 :1;
        unsigned LD6 :1;
        unsigned LD7 :1;
    };
} LATDbits_t;
extern volatile LATDbits_t LATDbits __attribute__((address(0xF8C)));
# 1158 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char LATE __attribute__((address(0xF8D)));

__asm("LATE equ 0F8Dh");


typedef union {
    struct {
        unsigned LATE0 :1;
        unsigned LATE1 :1;
        unsigned LATE2 :1;
    };
    struct {
        unsigned LE0 :1;
        unsigned LE1 :1;
        unsigned LE2 :1;
    };
} LATEbits_t;
extern volatile LATEbits_t LATEbits __attribute__((address(0xF8D)));
# 1210 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char TRISA __attribute__((address(0xF92)));

__asm("TRISA equ 0F92h");


extern volatile unsigned char DDRA __attribute__((address(0xF92)));

__asm("DDRA equ 0F92h");


typedef union {
    struct {
        unsigned TRISA0 :1;
        unsigned TRISA1 :1;
        unsigned TRISA2 :1;
        unsigned TRISA3 :1;
        unsigned TRISA4 :1;
        unsigned TRISA5 :1;
        unsigned TRISA6 :1;
    };
    struct {
        unsigned RA0 :1;
        unsigned RA1 :1;
        unsigned RA2 :1;
        unsigned RA3 :1;
        unsigned RA4 :1;
        unsigned RA5 :1;
        unsigned RA6 :1;
    };
} TRISAbits_t;
extern volatile TRISAbits_t TRISAbits __attribute__((address(0xF92)));
# 1313 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
typedef union {
    struct {
        unsigned TRISA0 :1;
        unsigned TRISA1 :1;
        unsigned TRISA2 :1;
        unsigned TRISA3 :1;
        unsigned TRISA4 :1;
        unsigned TRISA5 :1;
        unsigned TRISA6 :1;
    };
    struct {
        unsigned RA0 :1;
        unsigned RA1 :1;
        unsigned RA2 :1;
        unsigned RA3 :1;
        unsigned RA4 :1;
        unsigned RA5 :1;
        unsigned RA6 :1;
    };
} DDRAbits_t;
extern volatile DDRAbits_t DDRAbits __attribute__((address(0xF92)));
# 1408 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char TRISB __attribute__((address(0xF93)));

__asm("TRISB equ 0F93h");


extern volatile unsigned char DDRB __attribute__((address(0xF93)));

__asm("DDRB equ 0F93h");


typedef union {
    struct {
        unsigned TRISB0 :1;
        unsigned TRISB1 :1;
        unsigned TRISB2 :1;
        unsigned TRISB3 :1;
        unsigned TRISB4 :1;
        unsigned TRISB5 :1;
        unsigned TRISB6 :1;
        unsigned TRISB7 :1;
    };
    struct {
        unsigned RB0 :1;
        unsigned RB1 :1;
        unsigned RB2 :1;
        unsigned RB3 :1;
        unsigned RB4 :1;
        unsigned RB5 :1;
        unsigned RB6 :1;
        unsigned RB7 :1;
    };
    struct {
        unsigned :3;
        unsigned CCP2 :1;
    };
} TRISBbits_t;
extern volatile TRISBbits_t TRISBbits __attribute__((address(0xF93)));
# 1532 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
typedef union {
    struct {
        unsigned TRISB0 :1;
        unsigned TRISB1 :1;
        unsigned TRISB2 :1;
        unsigned TRISB3 :1;
        unsigned TRISB4 :1;
        unsigned TRISB5 :1;
        unsigned TRISB6 :1;
        unsigned TRISB7 :1;
    };
    struct {
        unsigned RB0 :1;
        unsigned RB1 :1;
        unsigned RB2 :1;
        unsigned RB3 :1;
        unsigned RB4 :1;
        unsigned RB5 :1;
        unsigned RB6 :1;
        unsigned RB7 :1;
    };
    struct {
        unsigned :3;
        unsigned CCP2 :1;
    };
} DDRBbits_t;
extern volatile DDRBbits_t DDRBbits __attribute__((address(0xF93)));
# 1648 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char TRISC __attribute__((address(0xF94)));

__asm("TRISC equ 0F94h");


extern volatile unsigned char DDRC __attribute__((address(0xF94)));

__asm("DDRC equ 0F94h");


typedef union {
    struct {
        unsigned TRISC0 :1;
        unsigned TRISC1 :1;
        unsigned TRISC2 :1;
        unsigned TRISC3 :1;
        unsigned TRISC4 :1;
        unsigned TRISC5 :1;
        unsigned TRISC6 :1;
        unsigned TRISC7 :1;
    };
    struct {
        unsigned RC0 :1;
        unsigned RC1 :1;
        unsigned RC2 :1;
        unsigned RC3 :1;
        unsigned RC4 :1;
        unsigned RC5 :1;
        unsigned RC6 :1;
        unsigned RC7 :1;
    };
    struct {
        unsigned :1;
        unsigned CCP2 :1;
    };
} TRISCbits_t;
extern volatile TRISCbits_t TRISCbits __attribute__((address(0xF94)));
# 1772 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
typedef union {
    struct {
        unsigned TRISC0 :1;
        unsigned TRISC1 :1;
        unsigned TRISC2 :1;
        unsigned TRISC3 :1;
        unsigned TRISC4 :1;
        unsigned TRISC5 :1;
        unsigned TRISC6 :1;
        unsigned TRISC7 :1;
    };
    struct {
        unsigned RC0 :1;
        unsigned RC1 :1;
        unsigned RC2 :1;
        unsigned RC3 :1;
        unsigned RC4 :1;
        unsigned RC5 :1;
        unsigned RC6 :1;
        unsigned RC7 :1;
    };
    struct {
        unsigned :1;
        unsigned CCP2 :1;
    };
} DDRCbits_t;
extern volatile DDRCbits_t DDRCbits __attribute__((address(0xF94)));
# 1888 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char TRISD __attribute__((address(0xF95)));

__asm("TRISD equ 0F95h");


extern volatile unsigned char DDRD __attribute__((address(0xF95)));

__asm("DDRD equ 0F95h");


typedef union {
    struct {
        unsigned TRISD0 :1;
        unsigned TRISD1 :1;
        unsigned TRISD2 :1;
        unsigned TRISD3 :1;
        unsigned TRISD4 :1;
        unsigned TRISD5 :1;
        unsigned TRISD6 :1;
        unsigned TRISD7 :1;
    };
    struct {
        unsigned RD0 :1;
        unsigned RD1 :1;
        unsigned RD2 :1;
        unsigned RD3 :1;
        unsigned RD4 :1;
        unsigned RD5 :1;
        unsigned RD6 :1;
        unsigned RD7 :1;
    };
} TRISDbits_t;
extern volatile TRISDbits_t TRISDbits __attribute__((address(0xF95)));
# 2003 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
typedef union {
    struct {
        unsigned TRISD0 :1;
        unsigned TRISD1 :1;
        unsigned TRISD2 :1;
        unsigned TRISD3 :1;
        unsigned TRISD4 :1;
        unsigned TRISD5 :1;
        unsigned TRISD6 :1;
        unsigned TRISD7 :1;
    };
    struct {
        unsigned RD0 :1;
        unsigned RD1 :1;
        unsigned RD2 :1;
        unsigned RD3 :1;
        unsigned RD4 :1;
        unsigned RD5 :1;
        unsigned RD6 :1;
        unsigned RD7 :1;
    };
} DDRDbits_t;
extern volatile DDRDbits_t DDRDbits __attribute__((address(0xF95)));
# 2110 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char TRISE __attribute__((address(0xF96)));

__asm("TRISE equ 0F96h");


extern volatile unsigned char DDRE __attribute__((address(0xF96)));

__asm("DDRE equ 0F96h");


typedef union {
    struct {
        unsigned TRISE0 :1;
        unsigned TRISE1 :1;
        unsigned TRISE2 :1;
        unsigned :1;
        unsigned PSPMODE :1;
        unsigned IBOV :1;
        unsigned OBF :1;
        unsigned IBF :1;
    };
    struct {
        unsigned RE0 :1;
        unsigned RE1 :1;
        unsigned RE2 :1;
    };
} TRISEbits_t;
extern volatile TRISEbits_t TRISEbits __attribute__((address(0xF96)));
# 2190 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
typedef union {
    struct {
        unsigned TRISE0 :1;
        unsigned TRISE1 :1;
        unsigned TRISE2 :1;
        unsigned :1;
        unsigned PSPMODE :1;
        unsigned IBOV :1;
        unsigned OBF :1;
        unsigned IBF :1;
    };
    struct {
        unsigned RE0 :1;
        unsigned RE1 :1;
        unsigned RE2 :1;
    };
} DDREbits_t;
extern volatile DDREbits_t DDREbits __attribute__((address(0xF96)));
# 2262 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char PIE1 __attribute__((address(0xF9D)));

__asm("PIE1 equ 0F9Dh");


typedef union {
    struct {
        unsigned TMR1IE :1;
        unsigned TMR2IE :1;
        unsigned CCP1IE :1;
        unsigned SSPIE :1;
        unsigned TXIE :1;
        unsigned RCIE :1;
        unsigned ADIE :1;
        unsigned PSPIE :1;
    };
    struct {
        unsigned :4;
        unsigned TX1IE :1;
        unsigned RC1IE :1;
    };
} PIE1bits_t;
extern volatile PIE1bits_t PIE1bits __attribute__((address(0xF9D)));
# 2339 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char PIR1 __attribute__((address(0xF9E)));

__asm("PIR1 equ 0F9Eh");


typedef union {
    struct {
        unsigned TMR1IF :1;
        unsigned TMR2IF :1;
        unsigned CCP1IF :1;
        unsigned SSPIF :1;
        unsigned TXIF :1;
        unsigned RCIF :1;
        unsigned ADIF :1;
        unsigned PSPIF :1;
    };
    struct {
        unsigned :4;
        unsigned TX1IF :1;
        unsigned RC1IF :1;
    };
} PIR1bits_t;
extern volatile PIR1bits_t PIR1bits __attribute__((address(0xF9E)));
# 2416 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char IPR1 __attribute__((address(0xF9F)));

__asm("IPR1 equ 0F9Fh");


typedef union {
    struct {
        unsigned TMR1IP :1;
        unsigned TMR2IP :1;
        unsigned CCP1IP :1;
        unsigned SSPIP :1;
        unsigned TXIP :1;
        unsigned RCIP :1;
        unsigned ADIP :1;
        unsigned PSPIP :1;
    };
    struct {
        unsigned :4;
        unsigned TX1IP :1;
        unsigned RC1IP :1;
    };
} IPR1bits_t;
extern volatile IPR1bits_t IPR1bits __attribute__((address(0xF9F)));
# 2493 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char PIE2 __attribute__((address(0xFA0)));

__asm("PIE2 equ 0FA0h");


typedef union {
    struct {
        unsigned CCP2IE :1;
        unsigned TMR3IE :1;
        unsigned LVDIE :1;
        unsigned BCLIE :1;
        unsigned EEIE :1;
    };
} PIE2bits_t;
extern volatile PIE2bits_t PIE2bits __attribute__((address(0xFA0)));
# 2537 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char PIR2 __attribute__((address(0xFA1)));

__asm("PIR2 equ 0FA1h");


typedef union {
    struct {
        unsigned CCP2IF :1;
        unsigned TMR3IF :1;
        unsigned LVDIF :1;
        unsigned BCLIF :1;
        unsigned EEIF :1;
    };
} PIR2bits_t;
extern volatile PIR2bits_t PIR2bits __attribute__((address(0xFA1)));
# 2581 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char IPR2 __attribute__((address(0xFA2)));

__asm("IPR2 equ 0FA2h");


typedef union {
    struct {
        unsigned CCP2IP :1;
        unsigned TMR3IP :1;
        unsigned LVDIP :1;
        unsigned BCLIP :1;
        unsigned EEIP :1;
    };
} IPR2bits_t;
extern volatile IPR2bits_t IPR2bits __attribute__((address(0xFA2)));
# 2625 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char EECON1 __attribute__((address(0xFA6)));

__asm("EECON1 equ 0FA6h");


typedef union {
    struct {
        unsigned RD :1;
        unsigned WR :1;
        unsigned WREN :1;
        unsigned WRERR :1;
        unsigned FREE :1;
        unsigned :1;
        unsigned CFGS :1;
        unsigned EEPGD :1;
    };
    struct {
        unsigned :6;
        unsigned EEFS :1;
    };
} EECON1bits_t;
extern volatile EECON1bits_t EECON1bits __attribute__((address(0xFA6)));
# 2691 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char EECON2 __attribute__((address(0xFA7)));

__asm("EECON2 equ 0FA7h");




extern volatile unsigned char EEDATA __attribute__((address(0xFA8)));

__asm("EEDATA equ 0FA8h");




extern volatile unsigned char EEADR __attribute__((address(0xFA9)));

__asm("EEADR equ 0FA9h");




extern volatile unsigned char RCSTA __attribute__((address(0xFAB)));

__asm("RCSTA equ 0FABh");


extern volatile unsigned char RCSTA1 __attribute__((address(0xFAB)));

__asm("RCSTA1 equ 0FABh");


typedef union {
    struct {
        unsigned RX9D :1;
        unsigned OERR :1;
        unsigned FERR :1;
        unsigned ADDEN :1;
        unsigned CREN :1;
        unsigned SREN :1;
        unsigned RX9 :1;
        unsigned SPEN :1;
    };
    struct {
        unsigned RCD8 :1;
        unsigned :5;
        unsigned RC8_9 :1;
    };
    struct {
        unsigned :6;
        unsigned NOT_RC8 :1;
    };
    struct {
        unsigned :6;
        unsigned nRC8 :1;
    };
    struct {
        unsigned :6;
        unsigned RC9 :1;
    };
    struct {
        unsigned :5;
        unsigned SRENA :1;
    };
} RCSTAbits_t;
extern volatile RCSTAbits_t RCSTAbits __attribute__((address(0xFAB)));
# 2828 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
typedef union {
    struct {
        unsigned RX9D :1;
        unsigned OERR :1;
        unsigned FERR :1;
        unsigned ADDEN :1;
        unsigned CREN :1;
        unsigned SREN :1;
        unsigned RX9 :1;
        unsigned SPEN :1;
    };
    struct {
        unsigned RCD8 :1;
        unsigned :5;
        unsigned RC8_9 :1;
    };
    struct {
        unsigned :6;
        unsigned NOT_RC8 :1;
    };
    struct {
        unsigned :6;
        unsigned nRC8 :1;
    };
    struct {
        unsigned :6;
        unsigned RC9 :1;
    };
    struct {
        unsigned :5;
        unsigned SRENA :1;
    };
} RCSTA1bits_t;
extern volatile RCSTA1bits_t RCSTA1bits __attribute__((address(0xFAB)));
# 2936 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char TXSTA __attribute__((address(0xFAC)));

__asm("TXSTA equ 0FACh");


extern volatile unsigned char TXSTA1 __attribute__((address(0xFAC)));

__asm("TXSTA1 equ 0FACh");


typedef union {
    struct {
        unsigned TX9D :1;
        unsigned TRMT :1;
        unsigned BRGH :1;
        unsigned :1;
        unsigned SYNC :1;
        unsigned TXEN :1;
        unsigned TX9 :1;
        unsigned CSRC :1;
    };
    struct {
        unsigned TXD8 :1;
        unsigned :5;
        unsigned TX8_9 :1;
    };
    struct {
        unsigned :6;
        unsigned NOT_TX8 :1;
    };
    struct {
        unsigned :6;
        unsigned nTX8 :1;
    };
    struct {
        unsigned TX9D1 :1;
        unsigned TRMT1 :1;
        unsigned BRGH1 :1;
        unsigned :1;
        unsigned SYNC1 :1;
        unsigned TXEN1 :1;
        unsigned TX91 :1;
        unsigned CSRC1 :1;
    };
} TXSTAbits_t;
extern volatile TXSTAbits_t TXSTAbits __attribute__((address(0xFAC)));
# 3074 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
typedef union {
    struct {
        unsigned TX9D :1;
        unsigned TRMT :1;
        unsigned BRGH :1;
        unsigned :1;
        unsigned SYNC :1;
        unsigned TXEN :1;
        unsigned TX9 :1;
        unsigned CSRC :1;
    };
    struct {
        unsigned TXD8 :1;
        unsigned :5;
        unsigned TX8_9 :1;
    };
    struct {
        unsigned :6;
        unsigned NOT_TX8 :1;
    };
    struct {
        unsigned :6;
        unsigned nTX8 :1;
    };
    struct {
        unsigned TX9D1 :1;
        unsigned TRMT1 :1;
        unsigned BRGH1 :1;
        unsigned :1;
        unsigned SYNC1 :1;
        unsigned TXEN1 :1;
        unsigned TX91 :1;
        unsigned CSRC1 :1;
    };
} TXSTA1bits_t;
extern volatile TXSTA1bits_t TXSTA1bits __attribute__((address(0xFAC)));
# 3204 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char TXREG __attribute__((address(0xFAD)));

__asm("TXREG equ 0FADh");


extern volatile unsigned char TXREG1 __attribute__((address(0xFAD)));

__asm("TXREG1 equ 0FADh");




extern volatile unsigned char RCREG __attribute__((address(0xFAE)));

__asm("RCREG equ 0FAEh");


extern volatile unsigned char RCREG1 __attribute__((address(0xFAE)));

__asm("RCREG1 equ 0FAEh");




extern volatile unsigned char SPBRG __attribute__((address(0xFAF)));

__asm("SPBRG equ 0FAFh");


extern volatile unsigned char SPBRG1 __attribute__((address(0xFAF)));

__asm("SPBRG1 equ 0FAFh");




extern volatile unsigned char T3CON __attribute__((address(0xFB1)));

__asm("T3CON equ 0FB1h");


typedef union {
    struct {
        unsigned :2;
        unsigned NOT_T3SYNC :1;
    };
    struct {
        unsigned TMR3ON :1;
        unsigned TMR3CS :1;
        unsigned nT3SYNC :1;
        unsigned T3CCP1 :1;
        unsigned T3CKPS :2;
        unsigned T3CCP2 :1;
        unsigned RD16 :1;
    };
    struct {
        unsigned :2;
        unsigned T3SYNC :1;
        unsigned :1;
        unsigned T3CKPS0 :1;
        unsigned T3CKPS1 :1;
    };
    struct {
        unsigned :2;
        unsigned T3INSYNC :1;
    };
    struct {
        unsigned :3;
        unsigned SOSCEN3 :1;
        unsigned :3;
        unsigned RD163 :1;
    };
    struct {
        unsigned :7;
        unsigned T3RD16 :1;
    };
} T3CONbits_t;
extern volatile T3CONbits_t T3CONbits __attribute__((address(0xFB1)));
# 3361 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned short TMR3 __attribute__((address(0xFB2)));

__asm("TMR3 equ 0FB2h");




extern volatile unsigned char TMR3L __attribute__((address(0xFB2)));

__asm("TMR3L equ 0FB2h");




extern volatile unsigned char TMR3H __attribute__((address(0xFB3)));

__asm("TMR3H equ 0FB3h");




extern volatile unsigned char CCP2CON __attribute__((address(0xFBA)));

__asm("CCP2CON equ 0FBAh");


typedef union {
    struct {
        unsigned CCP2M :4;
        unsigned DC2B :2;
    };
    struct {
        unsigned CCP2M0 :1;
        unsigned CCP2M1 :1;
        unsigned CCP2M2 :1;
        unsigned CCP2M3 :1;
        unsigned DC2B0 :1;
        unsigned DC2B1 :1;
    };
    struct {
        unsigned :4;
        unsigned CCP2Y :1;
        unsigned CCP2X :1;
    };
    struct {
        unsigned :5;
        unsigned DCCPX :1;
    };
} CCP2CONbits_t;
extern volatile CCP2CONbits_t CCP2CONbits __attribute__((address(0xFBA)));
# 3470 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned short CCPR2 __attribute__((address(0xFBB)));

__asm("CCPR2 equ 0FBBh");




extern volatile unsigned char CCPR2L __attribute__((address(0xFBB)));

__asm("CCPR2L equ 0FBBh");




extern volatile unsigned char CCPR2H __attribute__((address(0xFBC)));

__asm("CCPR2H equ 0FBCh");




extern volatile unsigned char CCP1CON __attribute__((address(0xFBD)));

__asm("CCP1CON equ 0FBDh");


typedef union {
    struct {
        unsigned CCP1M :4;
        unsigned DC1B :2;
    };
    struct {
        unsigned CCP1M0 :1;
        unsigned CCP1M1 :1;
        unsigned CCP1M2 :1;
        unsigned CCP1M3 :1;
        unsigned DC1B0 :1;
        unsigned DC1B1 :1;
    };
    struct {
        unsigned :4;
        unsigned CCP1Y :1;
        unsigned CCP1X :1;
    };
} CCP1CONbits_t;
extern volatile CCP1CONbits_t CCP1CONbits __attribute__((address(0xFBD)));
# 3570 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned short CCPR1 __attribute__((address(0xFBE)));

__asm("CCPR1 equ 0FBEh");




extern volatile unsigned char CCPR1L __attribute__((address(0xFBE)));

__asm("CCPR1L equ 0FBEh");




extern volatile unsigned char CCPR1H __attribute__((address(0xFBF)));

__asm("CCPR1H equ 0FBFh");




extern volatile unsigned char ADCON1 __attribute__((address(0xFC1)));

__asm("ADCON1 equ 0FC1h");


typedef union {
    struct {
        unsigned PCFG :4;
        unsigned :2;
        unsigned ADCS2 :1;
        unsigned ADFM :1;
    };
    struct {
        unsigned PCFG0 :1;
        unsigned PCFG1 :1;
        unsigned PCFG2 :1;
        unsigned PCFG3 :1;
    };
    struct {
        unsigned :3;
        unsigned CHSN3 :1;
    };
} ADCON1bits_t;
extern volatile ADCON1bits_t ADCON1bits __attribute__((address(0xFC1)));
# 3659 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char ADCON0 __attribute__((address(0xFC2)));

__asm("ADCON0 equ 0FC2h");


typedef union {
    struct {
        unsigned :2;
        unsigned GO_NOT_DONE :1;
    };
    struct {
        unsigned ADON :1;
        unsigned :1;
        unsigned GO_nDONE :1;
        unsigned CHS :3;
        unsigned ADCS :2;
    };
    struct {
        unsigned :2;
        unsigned GO :1;
        unsigned CHS0 :1;
        unsigned CHS1 :1;
        unsigned CHS2 :1;
        unsigned ADCS0 :1;
        unsigned ADCS1 :1;
    };
    struct {
        unsigned :2;
        unsigned NOT_DONE :1;
    };
    struct {
        unsigned :2;
        unsigned nDONE :1;
    };
    struct {
        unsigned :2;
        unsigned DONE :1;
    };
    struct {
        unsigned :2;
        unsigned GO_DONE :1;
    };
    struct {
        unsigned :7;
        unsigned ADCAL :1;
    };
    struct {
        unsigned :2;
        unsigned GODONE :1;
    };
} ADCON0bits_t;
extern volatile ADCON0bits_t ADCON0bits __attribute__((address(0xFC2)));
# 3800 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned short ADRES __attribute__((address(0xFC3)));

__asm("ADRES equ 0FC3h");




extern volatile unsigned char ADRESL __attribute__((address(0xFC3)));

__asm("ADRESL equ 0FC3h");




extern volatile unsigned char ADRESH __attribute__((address(0xFC4)));

__asm("ADRESH equ 0FC4h");




extern volatile unsigned char SSPCON2 __attribute__((address(0xFC5)));

__asm("SSPCON2 equ 0FC5h");


typedef union {
    struct {
        unsigned SEN :1;
        unsigned RSEN :1;
        unsigned PEN :1;
        unsigned RCEN :1;
        unsigned ACKEN :1;
        unsigned ACKDT :1;
        unsigned ACKSTAT :1;
        unsigned GCEN :1;
    };
} SSPCON2bits_t;
extern volatile SSPCON2bits_t SSPCON2bits __attribute__((address(0xFC5)));
# 3883 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char SSPCON1 __attribute__((address(0xFC6)));

__asm("SSPCON1 equ 0FC6h");


typedef union {
    struct {
        unsigned SSPM :4;
        unsigned CKP :1;
        unsigned SSPEN :1;
        unsigned SSPOV :1;
        unsigned WCOL :1;
    };
    struct {
        unsigned SSPM0 :1;
        unsigned SSPM1 :1;
        unsigned SSPM2 :1;
        unsigned SSPM3 :1;
    };
} SSPCON1bits_t;
extern volatile SSPCON1bits_t SSPCON1bits __attribute__((address(0xFC6)));
# 3953 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char SSPSTAT __attribute__((address(0xFC7)));

__asm("SSPSTAT equ 0FC7h");


typedef union {
    struct {
        unsigned :2;
        unsigned R_NOT_W :1;
    };
    struct {
        unsigned :5;
        unsigned D_NOT_A :1;
    };
    struct {
        unsigned BF :1;
        unsigned UA :1;
        unsigned R_nW :1;
        unsigned S :1;
        unsigned P :1;
        unsigned D_nA :1;
        unsigned CKE :1;
        unsigned SMP :1;
    };
    struct {
        unsigned :2;
        unsigned I2C_READ :1;
        unsigned I2C_START :1;
        unsigned I2C_STOP :1;
        unsigned I2C_DATA :1;
    };
    struct {
        unsigned :2;
        unsigned R :1;
        unsigned :2;
        unsigned D :1;
    };
    struct {
        unsigned :2;
        unsigned READ_WRITE :1;
        unsigned :2;
        unsigned DATA_ADDRESS :1;
    };
    struct {
        unsigned :2;
        unsigned NOT_WRITE :1;
    };
    struct {
        unsigned :5;
        unsigned NOT_ADDRESS :1;
    };
    struct {
        unsigned :2;
        unsigned nWRITE :1;
        unsigned :2;
        unsigned nADDRESS :1;
    };
    struct {
        unsigned :2;
        unsigned nW :1;
        unsigned :2;
        unsigned nA :1;
    };
    struct {
        unsigned :2;
        unsigned R_W :1;
        unsigned :2;
        unsigned D_A :1;
    };
    struct {
        unsigned :5;
        unsigned I2C_DAT :1;
    };
    struct {
        unsigned :2;
        unsigned RW :1;
        unsigned START :1;
        unsigned STOP :1;
        unsigned DA :1;
    };
    struct {
        unsigned :2;
        unsigned NOT_W :1;
        unsigned :2;
        unsigned NOT_A :1;
    };
} SSPSTATbits_t;
extern volatile SSPSTATbits_t SSPSTATbits __attribute__((address(0xFC7)));
# 4210 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char SSPADD __attribute__((address(0xFC8)));

__asm("SSPADD equ 0FC8h");




extern volatile unsigned char SSPBUF __attribute__((address(0xFC9)));

__asm("SSPBUF equ 0FC9h");




extern volatile unsigned char T2CON __attribute__((address(0xFCA)));

__asm("T2CON equ 0FCAh");


typedef union {
    struct {
        unsigned T2CKPS :2;
        unsigned TMR2ON :1;
        unsigned TOUTPS :4;
    };
    struct {
        unsigned T2CKPS0 :1;
        unsigned T2CKPS1 :1;
        unsigned :1;
        unsigned TOUTPS0 :1;
        unsigned TOUTPS1 :1;
        unsigned TOUTPS2 :1;
        unsigned TOUTPS3 :1;
    };
} T2CONbits_t;
extern volatile T2CONbits_t T2CONbits __attribute__((address(0xFCA)));
# 4295 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char PR2 __attribute__((address(0xFCB)));

__asm("PR2 equ 0FCBh");


extern volatile unsigned char MEMCON __attribute__((address(0xFCB)));

__asm("MEMCON equ 0FCBh");


typedef union {
    struct {
        unsigned :7;
        unsigned EBDIS :1;
    };
    struct {
        unsigned :4;
        unsigned WAIT0 :1;
    };
    struct {
        unsigned :5;
        unsigned WAIT1 :1;
    };
    struct {
        unsigned WM0 :1;
    };
    struct {
        unsigned :1;
        unsigned WM1 :1;
    };
} PR2bits_t;
extern volatile PR2bits_t PR2bits __attribute__((address(0xFCB)));
# 4354 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
typedef union {
    struct {
        unsigned :7;
        unsigned EBDIS :1;
    };
    struct {
        unsigned :4;
        unsigned WAIT0 :1;
    };
    struct {
        unsigned :5;
        unsigned WAIT1 :1;
    };
    struct {
        unsigned WM0 :1;
    };
    struct {
        unsigned :1;
        unsigned WM1 :1;
    };
} MEMCONbits_t;
extern volatile MEMCONbits_t MEMCONbits __attribute__((address(0xFCB)));
# 4405 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char TMR2 __attribute__((address(0xFCC)));

__asm("TMR2 equ 0FCCh");




extern volatile unsigned char T1CON __attribute__((address(0xFCD)));

__asm("T1CON equ 0FCDh");


typedef union {
    struct {
        unsigned :2;
        unsigned NOT_T1SYNC :1;
    };
    struct {
        unsigned TMR1ON :1;
        unsigned TMR1CS :1;
        unsigned nT1SYNC :1;
        unsigned T1OSCEN :1;
        unsigned T1CKPS :2;
        unsigned :1;
        unsigned RD16 :1;
    };
    struct {
        unsigned :2;
        unsigned T1SYNC :1;
        unsigned :1;
        unsigned T1CKPS0 :1;
        unsigned T1CKPS1 :1;
    };
    struct {
        unsigned :2;
        unsigned T1INSYNC :1;
    };
    struct {
        unsigned :3;
        unsigned SOSCEN :1;
        unsigned :3;
        unsigned T1RD16 :1;
    };
} T1CONbits_t;
extern volatile T1CONbits_t T1CONbits __attribute__((address(0xFCD)));
# 4519 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned short TMR1 __attribute__((address(0xFCE)));

__asm("TMR1 equ 0FCEh");




extern volatile unsigned char TMR1L __attribute__((address(0xFCE)));

__asm("TMR1L equ 0FCEh");




extern volatile unsigned char TMR1H __attribute__((address(0xFCF)));

__asm("TMR1H equ 0FCFh");




extern volatile unsigned char RCON __attribute__((address(0xFD0)));

__asm("RCON equ 0FD0h");


typedef union {
    struct {
        unsigned NOT_BOR :1;
    };
    struct {
        unsigned :1;
        unsigned NOT_POR :1;
    };
    struct {
        unsigned :2;
        unsigned NOT_PD :1;
    };
    struct {
        unsigned :3;
        unsigned NOT_TO :1;
    };
    struct {
        unsigned :4;
        unsigned NOT_RI :1;
    };
    struct {
        unsigned nBOR :1;
        unsigned nPOR :1;
        unsigned nPD :1;
        unsigned nTO :1;
        unsigned nRI :1;
        unsigned :2;
        unsigned IPEN :1;
    };
    struct {
        unsigned :7;
        unsigned NOT_IPEN :1;
    };
    struct {
        unsigned BOR :1;
        unsigned POR :1;
        unsigned PD :1;
        unsigned TO :1;
        unsigned RI :1;
        unsigned :2;
        unsigned nIPEN :1;
    };
} RCONbits_t;
extern volatile RCONbits_t RCONbits __attribute__((address(0xFD0)));
# 4683 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char WDTCON __attribute__((address(0xFD1)));

__asm("WDTCON equ 0FD1h");


typedef union {
    struct {
        unsigned SWDTEN :1;
    };
    struct {
        unsigned SWDTE :1;
    };
} WDTCONbits_t;
extern volatile WDTCONbits_t WDTCONbits __attribute__((address(0xFD1)));
# 4711 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char LVDCON __attribute__((address(0xFD2)));

__asm("LVDCON equ 0FD2h");


typedef union {
    struct {
        unsigned LVDL :4;
        unsigned LVDEN :1;
        unsigned IRVST :1;
    };
    struct {
        unsigned LVDL0 :1;
        unsigned LVDL1 :1;
        unsigned LVDL2 :1;
        unsigned LVDL3 :1;
    };
} LVDCONbits_t;
extern volatile LVDCONbits_t LVDCONbits __attribute__((address(0xFD2)));
# 4769 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char OSCCON __attribute__((address(0xFD3)));

__asm("OSCCON equ 0FD3h");


typedef union {
    struct {
        unsigned SCS :1;
    };
} OSCCONbits_t;
extern volatile OSCCONbits_t OSCCONbits __attribute__((address(0xFD3)));
# 4789 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char T0CON __attribute__((address(0xFD5)));

__asm("T0CON equ 0FD5h");


typedef union {
    struct {
        unsigned T0PS :3;
        unsigned PSA :1;
        unsigned T0SE :1;
        unsigned T0CS :1;
        unsigned T08BIT :1;
        unsigned TMR0ON :1;
    };
    struct {
        unsigned T0PS0 :1;
        unsigned T0PS1 :1;
        unsigned T0PS2 :1;
    };
} T0CONbits_t;
extern volatile T0CONbits_t T0CONbits __attribute__((address(0xFD5)));
# 4859 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned short TMR0 __attribute__((address(0xFD6)));

__asm("TMR0 equ 0FD6h");




extern volatile unsigned char TMR0L __attribute__((address(0xFD6)));

__asm("TMR0L equ 0FD6h");




extern volatile unsigned char TMR0H __attribute__((address(0xFD7)));

__asm("TMR0H equ 0FD7h");




extern volatile unsigned char STATUS __attribute__((address(0xFD8)));

__asm("STATUS equ 0FD8h");


typedef union {
    struct {
        unsigned C :1;
        unsigned DC :1;
        unsigned Z :1;
        unsigned OV :1;
        unsigned N :1;
    };
    struct {
        unsigned CARRY :1;
        unsigned :1;
        unsigned ZERO :1;
        unsigned OVERFLOW :1;
        unsigned NEGATIVE :1;
    };
} STATUSbits_t;
extern volatile STATUSbits_t STATUSbits __attribute__((address(0xFD8)));
# 4951 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned short FSR2 __attribute__((address(0xFD9)));

__asm("FSR2 equ 0FD9h");




extern volatile unsigned char FSR2L __attribute__((address(0xFD9)));

__asm("FSR2L equ 0FD9h");




extern volatile unsigned char FSR2H __attribute__((address(0xFDA)));

__asm("FSR2H equ 0FDAh");




extern volatile unsigned char PLUSW2 __attribute__((address(0xFDB)));

__asm("PLUSW2 equ 0FDBh");




extern volatile unsigned char PREINC2 __attribute__((address(0xFDC)));

__asm("PREINC2 equ 0FDCh");




extern volatile unsigned char POSTDEC2 __attribute__((address(0xFDD)));

__asm("POSTDEC2 equ 0FDDh");




extern volatile unsigned char POSTINC2 __attribute__((address(0xFDE)));

__asm("POSTINC2 equ 0FDEh");




extern volatile unsigned char INDF2 __attribute__((address(0xFDF)));

__asm("INDF2 equ 0FDFh");




extern volatile unsigned char BSR __attribute__((address(0xFE0)));

__asm("BSR equ 0FE0h");




extern volatile unsigned short FSR1 __attribute__((address(0xFE1)));

__asm("FSR1 equ 0FE1h");




extern volatile unsigned char FSR1L __attribute__((address(0xFE1)));

__asm("FSR1L equ 0FE1h");




extern volatile unsigned char FSR1H __attribute__((address(0xFE2)));

__asm("FSR1H equ 0FE2h");




extern volatile unsigned char PLUSW1 __attribute__((address(0xFE3)));

__asm("PLUSW1 equ 0FE3h");




extern volatile unsigned char PREINC1 __attribute__((address(0xFE4)));

__asm("PREINC1 equ 0FE4h");




extern volatile unsigned char POSTDEC1 __attribute__((address(0xFE5)));

__asm("POSTDEC1 equ 0FE5h");




extern volatile unsigned char POSTINC1 __attribute__((address(0xFE6)));

__asm("POSTINC1 equ 0FE6h");




extern volatile unsigned char INDF1 __attribute__((address(0xFE7)));

__asm("INDF1 equ 0FE7h");




extern volatile unsigned char WREG __attribute__((address(0xFE8)));

__asm("WREG equ 0FE8h");
# 5082 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned short FSR0 __attribute__((address(0xFE9)));

__asm("FSR0 equ 0FE9h");




extern volatile unsigned char FSR0L __attribute__((address(0xFE9)));

__asm("FSR0L equ 0FE9h");




extern volatile unsigned char FSR0H __attribute__((address(0xFEA)));

__asm("FSR0H equ 0FEAh");




extern volatile unsigned char PLUSW0 __attribute__((address(0xFEB)));

__asm("PLUSW0 equ 0FEBh");




extern volatile unsigned char PREINC0 __attribute__((address(0xFEC)));

__asm("PREINC0 equ 0FECh");




extern volatile unsigned char POSTDEC0 __attribute__((address(0xFED)));

__asm("POSTDEC0 equ 0FEDh");




extern volatile unsigned char POSTINC0 __attribute__((address(0xFEE)));

__asm("POSTINC0 equ 0FEEh");




extern volatile unsigned char INDF0 __attribute__((address(0xFEF)));

__asm("INDF0 equ 0FEFh");




extern volatile unsigned char INTCON3 __attribute__((address(0xFF0)));

__asm("INTCON3 equ 0FF0h");


typedef union {
    struct {
        unsigned INT1IF :1;
        unsigned INT2IF :1;
        unsigned :1;
        unsigned INT1IE :1;
        unsigned INT2IE :1;
        unsigned :1;
        unsigned INT1IP :1;
        unsigned INT2IP :1;
    };
    struct {
        unsigned INT1F :1;
        unsigned INT2F :1;
        unsigned :1;
        unsigned INT1E :1;
        unsigned INT2E :1;
        unsigned :1;
        unsigned INT1P :1;
        unsigned INT2P :1;
    };
} INTCON3bits_t;
extern volatile INTCON3bits_t INTCON3bits __attribute__((address(0xFF0)));
# 5230 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char INTCON2 __attribute__((address(0xFF1)));

__asm("INTCON2 equ 0FF1h");


typedef union {
    struct {
        unsigned :7;
        unsigned NOT_RBPU :1;
    };
    struct {
        unsigned RBIP :1;
        unsigned :1;
        unsigned TMR0IP :1;
        unsigned :1;
        unsigned INTEDG2 :1;
        unsigned INTEDG1 :1;
        unsigned INTEDG0 :1;
        unsigned nRBPU :1;
    };
    struct {
        unsigned :2;
        unsigned T0IP :1;
        unsigned :4;
        unsigned RBPU :1;
    };
} INTCON2bits_t;
extern volatile INTCON2bits_t INTCON2bits __attribute__((address(0xFF1)));
# 5307 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned char INTCON __attribute__((address(0xFF2)));

__asm("INTCON equ 0FF2h");


extern volatile unsigned char INTCON1 __attribute__((address(0xFF2)));

__asm("INTCON1 equ 0FF2h");


typedef union {
    struct {
        unsigned RBIF :1;
        unsigned INT0IF :1;
        unsigned TMR0IF :1;
        unsigned RBIE :1;
        unsigned INT0IE :1;
        unsigned TMR0IE :1;
        unsigned PEIE_GIEL :1;
        unsigned GIE_GIEH :1;
    };
    struct {
        unsigned :1;
        unsigned INT0F :1;
        unsigned T0IF :1;
        unsigned :1;
        unsigned INT0E :1;
        unsigned T0IE :1;
        unsigned PEIE :1;
        unsigned GIE :1;
    };
    struct {
        unsigned :6;
        unsigned GIEL :1;
        unsigned GIEH :1;
    };
} INTCONbits_t;
extern volatile INTCONbits_t INTCONbits __attribute__((address(0xFF2)));
# 5427 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
typedef union {
    struct {
        unsigned RBIF :1;
        unsigned INT0IF :1;
        unsigned TMR0IF :1;
        unsigned RBIE :1;
        unsigned INT0IE :1;
        unsigned TMR0IE :1;
        unsigned PEIE_GIEL :1;
        unsigned GIE_GIEH :1;
    };
    struct {
        unsigned :1;
        unsigned INT0F :1;
        unsigned T0IF :1;
        unsigned :1;
        unsigned INT0E :1;
        unsigned T0IE :1;
        unsigned PEIE :1;
        unsigned GIE :1;
    };
    struct {
        unsigned :6;
        unsigned GIEL :1;
        unsigned GIEH :1;
    };
} INTCON1bits_t;
extern volatile INTCON1bits_t INTCON1bits __attribute__((address(0xFF2)));
# 5539 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile unsigned short PROD __attribute__((address(0xFF3)));

__asm("PROD equ 0FF3h");




extern volatile unsigned char PRODL __attribute__((address(0xFF3)));

__asm("PRODL equ 0FF3h");




extern volatile unsigned char PRODH __attribute__((address(0xFF4)));

__asm("PRODH equ 0FF4h");




extern volatile unsigned char TABLAT __attribute__((address(0xFF5)));

__asm("TABLAT equ 0FF5h");





extern volatile __uint24 TBLPTR __attribute__((address(0xFF6)));


__asm("TBLPTR equ 0FF6h");




extern volatile unsigned char TBLPTRL __attribute__((address(0xFF6)));

__asm("TBLPTRL equ 0FF6h");




extern volatile unsigned char TBLPTRH __attribute__((address(0xFF7)));

__asm("TBLPTRH equ 0FF7h");




extern volatile unsigned char TBLPTRU __attribute__((address(0xFF8)));

__asm("TBLPTRU equ 0FF8h");





extern volatile __uint24 PCLAT __attribute__((address(0xFF9)));


__asm("PCLAT equ 0FF9h");



extern volatile __uint24 PC __attribute__((address(0xFF9)));


__asm("PC equ 0FF9h");




extern volatile unsigned char PCL __attribute__((address(0xFF9)));

__asm("PCL equ 0FF9h");




extern volatile unsigned char PCLATH __attribute__((address(0xFFA)));

__asm("PCLATH equ 0FFAh");




extern volatile unsigned char PCLATU __attribute__((address(0xFFB)));

__asm("PCLATU equ 0FFBh");




extern volatile unsigned char STKPTR __attribute__((address(0xFFC)));

__asm("STKPTR equ 0FFCh");


typedef union {
    struct {
        unsigned STKPTR :5;
        unsigned :1;
        unsigned STKUNF :1;
        unsigned STKFUL :1;
    };
    struct {
        unsigned STKPTR0 :1;
        unsigned STKPTR1 :1;
        unsigned STKPTR2 :1;
        unsigned STKPTR3 :1;
        unsigned STKPTR4 :1;
        unsigned :2;
        unsigned STKOVF :1;
    };
    struct {
        unsigned SP0 :1;
        unsigned SP1 :1;
        unsigned SP2 :1;
        unsigned SP3 :1;
        unsigned SP4 :1;
    };
} STKPTRbits_t;
extern volatile STKPTRbits_t STKPTRbits __attribute__((address(0xFFC)));
# 5739 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile __uint24 TOS __attribute__((address(0xFFD)));


__asm("TOS equ 0FFDh");




extern volatile unsigned char TOSL __attribute__((address(0xFFD)));

__asm("TOSL equ 0FFDh");




extern volatile unsigned char TOSH __attribute__((address(0xFFE)));

__asm("TOSH equ 0FFEh");




extern volatile unsigned char TOSU __attribute__((address(0xFFF)));

__asm("TOSU equ 0FFFh");
# 5774 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/proc/pic18f452.h" 3
extern volatile __bit ACKDT __attribute__((address(0x7E2D)));


extern volatile __bit ACKEN __attribute__((address(0x7E2C)));


extern volatile __bit ACKSTAT __attribute__((address(0x7E2E)));


extern volatile __bit ADCAL __attribute__((address(0x7E17)));


extern volatile __bit ADCS0 __attribute__((address(0x7E16)));


extern volatile __bit ADCS1 __attribute__((address(0x7E17)));


extern volatile __bit ADCS2 __attribute__((address(0x7E0E)));


extern volatile __bit ADDEN __attribute__((address(0x7D5B)));


extern volatile __bit ADFM __attribute__((address(0x7E0F)));


extern volatile __bit ADIE __attribute__((address(0x7CEE)));


extern volatile __bit ADIF __attribute__((address(0x7CF6)));


extern volatile __bit ADIP __attribute__((address(0x7CFE)));


extern volatile __bit ADON __attribute__((address(0x7E10)));


extern volatile __bit AN0 __attribute__((address(0x7C00)));


extern volatile __bit AN1 __attribute__((address(0x7C01)));


extern volatile __bit AN2 __attribute__((address(0x7C02)));


extern volatile __bit AN3 __attribute__((address(0x7C03)));


extern volatile __bit AN4 __attribute__((address(0x7C05)));


extern volatile __bit AN5 __attribute__((address(0x7C20)));


extern volatile __bit AN6 __attribute__((address(0x7C21)));


extern volatile __bit AN7 __attribute__((address(0x7C22)));


extern volatile __bit BCLIE __attribute__((address(0x7D03)));


extern volatile __bit BCLIF __attribute__((address(0x7D0B)));


extern volatile __bit BCLIP __attribute__((address(0x7D13)));


extern volatile __bit BF __attribute__((address(0x7E38)));


extern volatile __bit BOR __attribute__((address(0x7E80)));


extern volatile __bit BRGH __attribute__((address(0x7D62)));


extern volatile __bit BRGH1 __attribute__((address(0x7D62)));


extern volatile __bit CARRY __attribute__((address(0x7EC0)));


extern volatile __bit CCP1 __attribute__((address(0x7C12)));


extern volatile __bit CCP10 __attribute__((address(0x7C22)));


extern volatile __bit CCP1IE __attribute__((address(0x7CEA)));


extern volatile __bit CCP1IF __attribute__((address(0x7CF2)));


extern volatile __bit CCP1IP __attribute__((address(0x7CFA)));


extern volatile __bit CCP1M0 __attribute__((address(0x7DE8)));


extern volatile __bit CCP1M1 __attribute__((address(0x7DE9)));


extern volatile __bit CCP1M2 __attribute__((address(0x7DEA)));


extern volatile __bit CCP1M3 __attribute__((address(0x7DEB)));


extern volatile __bit CCP1X __attribute__((address(0x7DED)));


extern volatile __bit CCP1Y __attribute__((address(0x7DEC)));


extern volatile __bit CCP2A __attribute__((address(0x7C0B)));


extern volatile __bit CCP2IE __attribute__((address(0x7D00)));


extern volatile __bit CCP2IF __attribute__((address(0x7D08)));


extern volatile __bit CCP2IP __attribute__((address(0x7D10)));


extern volatile __bit CCP2M0 __attribute__((address(0x7DD0)));


extern volatile __bit CCP2M1 __attribute__((address(0x7DD1)));


extern volatile __bit CCP2M2 __attribute__((address(0x7DD2)));


extern volatile __bit CCP2M3 __attribute__((address(0x7DD3)));


extern volatile __bit CCP2X __attribute__((address(0x7DD5)));


extern volatile __bit CCP2Y __attribute__((address(0x7DD4)));


extern volatile __bit CCP2_PA2 __attribute__((address(0x7C0B)));


extern volatile __bit CFGS __attribute__((address(0x7D36)));


extern volatile __bit CHS0 __attribute__((address(0x7E13)));


extern volatile __bit CHS1 __attribute__((address(0x7E14)));


extern volatile __bit CHS2 __attribute__((address(0x7E15)));


extern volatile __bit CHSN3 __attribute__((address(0x7E0B)));


extern volatile __bit CK __attribute__((address(0x7C16)));


extern volatile __bit CKE __attribute__((address(0x7E3E)));


extern volatile __bit CKP __attribute__((address(0x7E34)));


extern volatile __bit CLKO __attribute__((address(0x7C06)));


extern volatile __bit CREN __attribute__((address(0x7D5C)));


extern volatile __bit CS __attribute__((address(0x7C22)));


extern volatile __bit CSRC __attribute__((address(0x7D67)));


extern volatile __bit CSRC1 __attribute__((address(0x7D67)));


extern volatile __bit DA __attribute__((address(0x7E3D)));


extern volatile __bit DATA_ADDRESS __attribute__((address(0x7E3D)));


extern volatile __bit DC __attribute__((address(0x7EC1)));


extern volatile __bit DC1B0 __attribute__((address(0x7DEC)));


extern volatile __bit DC1B1 __attribute__((address(0x7DED)));


extern volatile __bit DC2B0 __attribute__((address(0x7DD4)));


extern volatile __bit DC2B1 __attribute__((address(0x7DD5)));


extern volatile __bit DCCPX __attribute__((address(0x7DD5)));


extern volatile __bit DONE __attribute__((address(0x7E12)));


extern volatile __bit DT __attribute__((address(0x7C17)));


extern volatile __bit D_A __attribute__((address(0x7E3D)));


extern volatile __bit D_NOT_A __attribute__((address(0x7E3D)));


extern volatile __bit D_nA __attribute__((address(0x7E3D)));


extern volatile __bit EBDIS __attribute__((address(0x7E5F)));


extern volatile __bit EEFS __attribute__((address(0x7D36)));


extern volatile __bit EEIE __attribute__((address(0x7D04)));


extern volatile __bit EEIF __attribute__((address(0x7D0C)));


extern volatile __bit EEIP __attribute__((address(0x7D14)));


extern volatile __bit EEPGD __attribute__((address(0x7D37)));


extern volatile __bit FERR __attribute__((address(0x7D5A)));


extern volatile __bit FREE __attribute__((address(0x7D34)));


extern volatile __bit GCEN __attribute__((address(0x7E2F)));


extern volatile __bit GIE __attribute__((address(0x7F97)));


extern volatile __bit GIEH __attribute__((address(0x7F97)));


extern volatile __bit GIEL __attribute__((address(0x7F96)));


extern volatile __bit GIE_GIEH __attribute__((address(0x7F97)));


extern volatile __bit GO __attribute__((address(0x7E12)));


extern volatile __bit GODONE __attribute__((address(0x7E12)));


extern volatile __bit GO_DONE __attribute__((address(0x7E12)));


extern volatile __bit GO_NOT_DONE __attribute__((address(0x7E12)));


extern volatile __bit GO_nDONE __attribute__((address(0x7E12)));


extern volatile __bit I2C_DAT __attribute__((address(0x7E3D)));


extern volatile __bit I2C_DATA __attribute__((address(0x7E3D)));


extern volatile __bit I2C_READ __attribute__((address(0x7E3A)));


extern volatile __bit I2C_START __attribute__((address(0x7E3B)));


extern volatile __bit I2C_STOP __attribute__((address(0x7E3C)));


extern volatile __bit IBF __attribute__((address(0x7CB7)));


extern volatile __bit IBOV __attribute__((address(0x7CB5)));


extern volatile __bit INT0 __attribute__((address(0x7C08)));


extern volatile __bit INT0E __attribute__((address(0x7F94)));


extern volatile __bit INT0F __attribute__((address(0x7F91)));


extern volatile __bit INT0IE __attribute__((address(0x7F94)));


extern volatile __bit INT0IF __attribute__((address(0x7F91)));


extern volatile __bit INT1 __attribute__((address(0x7C09)));


extern volatile __bit INT1E __attribute__((address(0x7F83)));


extern volatile __bit INT1F __attribute__((address(0x7F80)));


extern volatile __bit INT1IE __attribute__((address(0x7F83)));


extern volatile __bit INT1IF __attribute__((address(0x7F80)));


extern volatile __bit INT1IP __attribute__((address(0x7F86)));


extern volatile __bit INT1P __attribute__((address(0x7F86)));


extern volatile __bit INT2 __attribute__((address(0x7C0A)));


extern volatile __bit INT2E __attribute__((address(0x7F84)));


extern volatile __bit INT2F __attribute__((address(0x7F81)));


extern volatile __bit INT2IE __attribute__((address(0x7F84)));


extern volatile __bit INT2IF __attribute__((address(0x7F81)));


extern volatile __bit INT2IP __attribute__((address(0x7F87)));


extern volatile __bit INT2P __attribute__((address(0x7F87)));


extern volatile __bit INTEDG0 __attribute__((address(0x7F8E)));


extern volatile __bit INTEDG1 __attribute__((address(0x7F8D)));


extern volatile __bit INTEDG2 __attribute__((address(0x7F8C)));


extern volatile __bit IPEN __attribute__((address(0x7E87)));


extern volatile __bit IRVST __attribute__((address(0x7E95)));


extern volatile __bit LA0 __attribute__((address(0x7C48)));


extern volatile __bit LA1 __attribute__((address(0x7C49)));


extern volatile __bit LA2 __attribute__((address(0x7C4A)));


extern volatile __bit LA3 __attribute__((address(0x7C4B)));


extern volatile __bit LA4 __attribute__((address(0x7C4C)));


extern volatile __bit LA5 __attribute__((address(0x7C4D)));


extern volatile __bit LA6 __attribute__((address(0x7C4E)));


extern volatile __bit LATA0 __attribute__((address(0x7C48)));


extern volatile __bit LATA1 __attribute__((address(0x7C49)));


extern volatile __bit LATA2 __attribute__((address(0x7C4A)));


extern volatile __bit LATA3 __attribute__((address(0x7C4B)));


extern volatile __bit LATA4 __attribute__((address(0x7C4C)));


extern volatile __bit LATA5 __attribute__((address(0x7C4D)));


extern volatile __bit LATA6 __attribute__((address(0x7C4E)));


extern volatile __bit LATB0 __attribute__((address(0x7C50)));


extern volatile __bit LATB1 __attribute__((address(0x7C51)));


extern volatile __bit LATB2 __attribute__((address(0x7C52)));


extern volatile __bit LATB3 __attribute__((address(0x7C53)));


extern volatile __bit LATB4 __attribute__((address(0x7C54)));


extern volatile __bit LATB5 __attribute__((address(0x7C55)));


extern volatile __bit LATB6 __attribute__((address(0x7C56)));


extern volatile __bit LATB7 __attribute__((address(0x7C57)));


extern volatile __bit LATC0 __attribute__((address(0x7C58)));


extern volatile __bit LATC1 __attribute__((address(0x7C59)));


extern volatile __bit LATC2 __attribute__((address(0x7C5A)));


extern volatile __bit LATC3 __attribute__((address(0x7C5B)));


extern volatile __bit LATC4 __attribute__((address(0x7C5C)));


extern volatile __bit LATC5 __attribute__((address(0x7C5D)));


extern volatile __bit LATC6 __attribute__((address(0x7C5E)));


extern volatile __bit LATC7 __attribute__((address(0x7C5F)));


extern volatile __bit LATD0 __attribute__((address(0x7C60)));


extern volatile __bit LATD1 __attribute__((address(0x7C61)));


extern volatile __bit LATD2 __attribute__((address(0x7C62)));


extern volatile __bit LATD3 __attribute__((address(0x7C63)));


extern volatile __bit LATD4 __attribute__((address(0x7C64)));


extern volatile __bit LATD5 __attribute__((address(0x7C65)));


extern volatile __bit LATD6 __attribute__((address(0x7C66)));


extern volatile __bit LATD7 __attribute__((address(0x7C67)));


extern volatile __bit LATE0 __attribute__((address(0x7C68)));


extern volatile __bit LATE1 __attribute__((address(0x7C69)));


extern volatile __bit LATE2 __attribute__((address(0x7C6A)));


extern volatile __bit LB0 __attribute__((address(0x7C50)));


extern volatile __bit LB1 __attribute__((address(0x7C51)));


extern volatile __bit LB2 __attribute__((address(0x7C52)));


extern volatile __bit LB3 __attribute__((address(0x7C53)));


extern volatile __bit LB4 __attribute__((address(0x7C54)));


extern volatile __bit LB5 __attribute__((address(0x7C55)));


extern volatile __bit LB6 __attribute__((address(0x7C56)));


extern volatile __bit LB7 __attribute__((address(0x7C57)));


extern volatile __bit LC0 __attribute__((address(0x7C58)));


extern volatile __bit LC1 __attribute__((address(0x7C59)));


extern volatile __bit LC2 __attribute__((address(0x7C5A)));


extern volatile __bit LC3 __attribute__((address(0x7C5B)));


extern volatile __bit LC4 __attribute__((address(0x7C5C)));


extern volatile __bit LC5 __attribute__((address(0x7C5D)));


extern volatile __bit LC6 __attribute__((address(0x7C5E)));


extern volatile __bit LC7 __attribute__((address(0x7C5F)));


extern volatile __bit LD0 __attribute__((address(0x7C60)));


extern volatile __bit LD1 __attribute__((address(0x7C61)));


extern volatile __bit LD2 __attribute__((address(0x7C62)));


extern volatile __bit LD3 __attribute__((address(0x7C63)));


extern volatile __bit LD4 __attribute__((address(0x7C64)));


extern volatile __bit LD5 __attribute__((address(0x7C65)));


extern volatile __bit LD6 __attribute__((address(0x7C66)));


extern volatile __bit LD7 __attribute__((address(0x7C67)));


extern volatile __bit LE0 __attribute__((address(0x7C68)));


extern volatile __bit LE1 __attribute__((address(0x7C69)));


extern volatile __bit LE2 __attribute__((address(0x7C6A)));


extern volatile __bit LVDEN __attribute__((address(0x7E94)));


extern volatile __bit LVDIE __attribute__((address(0x7D02)));


extern volatile __bit LVDIF __attribute__((address(0x7D0A)));


extern volatile __bit LVDIN __attribute__((address(0x7C05)));


extern volatile __bit LVDIP __attribute__((address(0x7D12)));


extern volatile __bit LVDL0 __attribute__((address(0x7E90)));


extern volatile __bit LVDL1 __attribute__((address(0x7E91)));


extern volatile __bit LVDL2 __attribute__((address(0x7E92)));


extern volatile __bit LVDL3 __attribute__((address(0x7E93)));


extern volatile __bit NEGATIVE __attribute__((address(0x7EC4)));


extern volatile __bit NOT_A __attribute__((address(0x7E3D)));


extern volatile __bit NOT_ADDRESS __attribute__((address(0x7E3D)));


extern volatile __bit NOT_BOR __attribute__((address(0x7E80)));


extern volatile __bit NOT_DONE __attribute__((address(0x7E12)));


extern volatile __bit NOT_IPEN __attribute__((address(0x7E87)));


extern volatile __bit NOT_PD __attribute__((address(0x7E82)));


extern volatile __bit NOT_POR __attribute__((address(0x7E81)));


extern volatile __bit NOT_RBPU __attribute__((address(0x7F8F)));


extern volatile __bit NOT_RC8 __attribute__((address(0x7D5E)));


extern volatile __bit NOT_RI __attribute__((address(0x7E84)));


extern volatile __bit NOT_T1SYNC __attribute__((address(0x7E6A)));


extern volatile __bit NOT_T3SYNC __attribute__((address(0x7D8A)));


extern volatile __bit NOT_TO __attribute__((address(0x7E83)));


extern volatile __bit NOT_TX8 __attribute__((address(0x7D66)));


extern volatile __bit NOT_W __attribute__((address(0x7E3A)));


extern volatile __bit NOT_WRITE __attribute__((address(0x7E3A)));


extern volatile __bit OBF __attribute__((address(0x7CB6)));


extern volatile __bit OERR __attribute__((address(0x7D59)));


extern volatile __bit OSC2 __attribute__((address(0x7C06)));


extern volatile __bit OV __attribute__((address(0x7EC3)));


extern volatile __bit OVERFLOW __attribute__((address(0x7EC3)));


extern volatile __bit PA1 __attribute__((address(0x7C12)));


extern volatile __bit PA2 __attribute__((address(0x7C11)));


extern volatile __bit PB2 __attribute__((address(0x7C22)));


extern volatile __bit PC2 __attribute__((address(0x7C21)));


extern volatile __bit PCFG0 __attribute__((address(0x7E08)));


extern volatile __bit PCFG1 __attribute__((address(0x7E09)));


extern volatile __bit PCFG2 __attribute__((address(0x7E0A)));


extern volatile __bit PCFG3 __attribute__((address(0x7E0B)));


extern volatile __bit PD __attribute__((address(0x7E82)));


extern volatile __bit PD2 __attribute__((address(0x7C20)));


extern volatile __bit PEIE __attribute__((address(0x7F96)));


extern volatile __bit PEIE_GIEL __attribute__((address(0x7F96)));


extern volatile __bit PEN __attribute__((address(0x7E2A)));


extern volatile __bit PGC __attribute__((address(0x7C0E)));


extern volatile __bit PGD __attribute__((address(0x7C0F)));


extern volatile __bit PGM __attribute__((address(0x7C0D)));


extern volatile __bit POR __attribute__((address(0x7E81)));


extern volatile __bit PSA __attribute__((address(0x7EAB)));


extern volatile __bit PSP0 __attribute__((address(0x7C18)));


extern volatile __bit PSP1 __attribute__((address(0x7C19)));


extern volatile __bit PSP2 __attribute__((address(0x7C1A)));


extern volatile __bit PSP3 __attribute__((address(0x7C1B)));


extern volatile __bit PSP4 __attribute__((address(0x7C1C)));


extern volatile __bit PSP5 __attribute__((address(0x7C1D)));


extern volatile __bit PSP6 __attribute__((address(0x7C1E)));


extern volatile __bit PSP7 __attribute__((address(0x7C1F)));


extern volatile __bit PSPIE __attribute__((address(0x7CEF)));


extern volatile __bit PSPIF __attribute__((address(0x7CF7)));


extern volatile __bit PSPIP __attribute__((address(0x7CFF)));


extern volatile __bit PSPMODE __attribute__((address(0x7CB4)));


extern volatile __bit __attribute__((__deprecated__)) RA0 __attribute__((address(0x7C00)));


extern volatile __bit __attribute__((__deprecated__)) RA1 __attribute__((address(0x7C01)));


extern volatile __bit __attribute__((__deprecated__)) RA2 __attribute__((address(0x7C02)));


extern volatile __bit __attribute__((__deprecated__)) RA3 __attribute__((address(0x7C03)));


extern volatile __bit __attribute__((__deprecated__)) RA4 __attribute__((address(0x7C04)));


extern volatile __bit __attribute__((__deprecated__)) RA5 __attribute__((address(0x7C05)));


extern volatile __bit __attribute__((__deprecated__)) RA6 __attribute__((address(0x7C06)));


extern volatile __bit __attribute__((__deprecated__)) RB0 __attribute__((address(0x7C08)));


extern volatile __bit __attribute__((__deprecated__)) RB1 __attribute__((address(0x7C09)));


extern volatile __bit __attribute__((__deprecated__)) RB2 __attribute__((address(0x7C0A)));


extern volatile __bit __attribute__((__deprecated__)) RB3 __attribute__((address(0x7C0B)));


extern volatile __bit __attribute__((__deprecated__)) RB4 __attribute__((address(0x7C0C)));


extern volatile __bit __attribute__((__deprecated__)) RB5 __attribute__((address(0x7C0D)));


extern volatile __bit __attribute__((__deprecated__)) RB6 __attribute__((address(0x7C0E)));


extern volatile __bit __attribute__((__deprecated__)) RB7 __attribute__((address(0x7C0F)));


extern volatile __bit RBIE __attribute__((address(0x7F93)));


extern volatile __bit RBIF __attribute__((address(0x7F90)));


extern volatile __bit RBIP __attribute__((address(0x7F88)));


extern volatile __bit RBPU __attribute__((address(0x7F8F)));


extern volatile __bit __attribute__((__deprecated__)) RC0 __attribute__((address(0x7C10)));


extern volatile __bit __attribute__((__deprecated__)) RC1 __attribute__((address(0x7C11)));


extern volatile __bit RC1IE __attribute__((address(0x7CED)));


extern volatile __bit RC1IF __attribute__((address(0x7CF5)));


extern volatile __bit RC1IP __attribute__((address(0x7CFD)));


extern volatile __bit __attribute__((__deprecated__)) RC2 __attribute__((address(0x7C12)));


extern volatile __bit __attribute__((__deprecated__)) RC3 __attribute__((address(0x7C13)));


extern volatile __bit __attribute__((__deprecated__)) RC4 __attribute__((address(0x7C14)));


extern volatile __bit __attribute__((__deprecated__)) RC5 __attribute__((address(0x7C15)));


extern volatile __bit __attribute__((__deprecated__)) RC6 __attribute__((address(0x7C16)));


extern volatile __bit __attribute__((__deprecated__)) RC7 __attribute__((address(0x7C17)));


extern volatile __bit RC8_9 __attribute__((address(0x7D5E)));


extern volatile __bit RC9 __attribute__((address(0x7D5E)));


extern volatile __bit RCD8 __attribute__((address(0x7D58)));


extern volatile __bit RCEN __attribute__((address(0x7E2B)));


extern volatile __bit RCIE __attribute__((address(0x7CED)));


extern volatile __bit RCIF __attribute__((address(0x7CF5)));


extern volatile __bit RCIP __attribute__((address(0x7CFD)));


extern volatile __bit __attribute__((__deprecated__)) RD __attribute__((address(0x7D30)));


extern volatile __bit __attribute__((__deprecated__)) RD0 __attribute__((address(0x7C18)));


extern volatile __bit __attribute__((__deprecated__)) RD1 __attribute__((address(0x7C19)));


extern volatile __bit __attribute__((__deprecated__)) RD16 __attribute__((address(0x7E6F)));


extern volatile __bit RD163 __attribute__((address(0x7D8F)));


extern volatile __bit __attribute__((__deprecated__)) RD2 __attribute__((address(0x7C1A)));


extern volatile __bit __attribute__((__deprecated__)) RD3 __attribute__((address(0x7C1B)));


extern volatile __bit __attribute__((__deprecated__)) RD4 __attribute__((address(0x7C1C)));


extern volatile __bit __attribute__((__deprecated__)) RD5 __attribute__((address(0x7C1D)));


extern volatile __bit __attribute__((__deprecated__)) RD6 __attribute__((address(0x7C1E)));


extern volatile __bit __attribute__((__deprecated__)) RD7 __attribute__((address(0x7C1F)));


extern volatile __bit RDE __attribute__((address(0x7C20)));


extern volatile __bit __attribute__((__deprecated__)) RE0 __attribute__((address(0x7C20)));


extern volatile __bit __attribute__((__deprecated__)) RE1 __attribute__((address(0x7C21)));


extern volatile __bit __attribute__((__deprecated__)) RE2 __attribute__((address(0x7C22)));


extern volatile __bit READ_WRITE __attribute__((address(0x7E3A)));


extern volatile __bit RI __attribute__((address(0x7E84)));


extern volatile __bit RSEN __attribute__((address(0x7E29)));


extern volatile __bit RW __attribute__((address(0x7E3A)));


extern volatile __bit RX __attribute__((address(0x7C17)));


extern volatile __bit RX9 __attribute__((address(0x7D5E)));


extern volatile __bit RX9D __attribute__((address(0x7D58)));


extern volatile __bit R_NOT_W __attribute__((address(0x7E3A)));


extern volatile __bit R_W __attribute__((address(0x7E3A)));


extern volatile __bit R_nW __attribute__((address(0x7E3A)));


extern volatile __bit SCK __attribute__((address(0x7C13)));


extern volatile __bit SCL __attribute__((address(0x7C13)));


extern volatile __bit SCS __attribute__((address(0x7E98)));


extern volatile __bit SDA __attribute__((address(0x7C14)));


extern volatile __bit SDI __attribute__((address(0x7C14)));


extern volatile __bit SDO __attribute__((address(0x7C15)));


extern volatile __bit SEN __attribute__((address(0x7E28)));


extern volatile __bit SMP __attribute__((address(0x7E3F)));


extern volatile __bit SOSCEN __attribute__((address(0x7E6B)));


extern volatile __bit SOSCEN3 __attribute__((address(0x7D8B)));


extern volatile __bit SP0 __attribute__((address(0x7FE0)));


extern volatile __bit SP1 __attribute__((address(0x7FE1)));


extern volatile __bit SP2 __attribute__((address(0x7FE2)));


extern volatile __bit SP3 __attribute__((address(0x7FE3)));


extern volatile __bit SP4 __attribute__((address(0x7FE4)));


extern volatile __bit SPEN __attribute__((address(0x7D5F)));


extern volatile __bit SREN __attribute__((address(0x7D5D)));


extern volatile __bit SRENA __attribute__((address(0x7D5D)));


extern volatile __bit SS __attribute__((address(0x7C05)));


extern volatile __bit SS2 __attribute__((address(0x7C1F)));


extern volatile __bit SSPEN __attribute__((address(0x7E35)));


extern volatile __bit SSPIE __attribute__((address(0x7CEB)));


extern volatile __bit SSPIF __attribute__((address(0x7CF3)));


extern volatile __bit SSPIP __attribute__((address(0x7CFB)));


extern volatile __bit SSPM0 __attribute__((address(0x7E30)));


extern volatile __bit SSPM1 __attribute__((address(0x7E31)));


extern volatile __bit SSPM2 __attribute__((address(0x7E32)));


extern volatile __bit SSPM3 __attribute__((address(0x7E33)));


extern volatile __bit SSPOV __attribute__((address(0x7E36)));


extern volatile __bit START __attribute__((address(0x7E3B)));


extern volatile __bit STKFUL __attribute__((address(0x7FE7)));


extern volatile __bit STKOVF __attribute__((address(0x7FE7)));


extern volatile __bit STKPTR0 __attribute__((address(0x7FE0)));


extern volatile __bit STKPTR1 __attribute__((address(0x7FE1)));


extern volatile __bit STKPTR2 __attribute__((address(0x7FE2)));


extern volatile __bit STKPTR3 __attribute__((address(0x7FE3)));


extern volatile __bit STKPTR4 __attribute__((address(0x7FE4)));


extern volatile __bit STKUNF __attribute__((address(0x7FE6)));


extern volatile __bit STOP __attribute__((address(0x7E3C)));


extern volatile __bit SWDTE __attribute__((address(0x7E88)));


extern volatile __bit SWDTEN __attribute__((address(0x7E88)));


extern volatile __bit SYNC __attribute__((address(0x7D64)));


extern volatile __bit SYNC1 __attribute__((address(0x7D64)));


extern volatile __bit T08BIT __attribute__((address(0x7EAE)));


extern volatile __bit T0CKI __attribute__((address(0x7C04)));


extern volatile __bit T0CS __attribute__((address(0x7EAD)));


extern volatile __bit T0IE __attribute__((address(0x7F95)));


extern volatile __bit T0IF __attribute__((address(0x7F92)));


extern volatile __bit T0IP __attribute__((address(0x7F8A)));


extern volatile __bit T0PS0 __attribute__((address(0x7EA8)));


extern volatile __bit T0PS1 __attribute__((address(0x7EA9)));


extern volatile __bit T0PS2 __attribute__((address(0x7EAA)));


extern volatile __bit T0SE __attribute__((address(0x7EAC)));


extern volatile __bit T1CKI __attribute__((address(0x7C10)));


extern volatile __bit T1CKPS0 __attribute__((address(0x7E6C)));


extern volatile __bit T1CKPS1 __attribute__((address(0x7E6D)));


extern volatile __bit T1INSYNC __attribute__((address(0x7E6A)));


extern volatile __bit T1OSCEN __attribute__((address(0x7E6B)));


extern volatile __bit T1OSI __attribute__((address(0x7C11)));


extern volatile __bit T1OSO __attribute__((address(0x7C10)));


extern volatile __bit T1RD16 __attribute__((address(0x7E6F)));


extern volatile __bit T1SYNC __attribute__((address(0x7E6A)));


extern volatile __bit T2CKPS0 __attribute__((address(0x7E50)));


extern volatile __bit T2CKPS1 __attribute__((address(0x7E51)));


extern volatile __bit T3CCP1 __attribute__((address(0x7D8B)));


extern volatile __bit T3CCP2 __attribute__((address(0x7D8E)));


extern volatile __bit T3CKPS0 __attribute__((address(0x7D8C)));


extern volatile __bit T3CKPS1 __attribute__((address(0x7D8D)));


extern volatile __bit T3INSYNC __attribute__((address(0x7D8A)));


extern volatile __bit T3RD16 __attribute__((address(0x7D8F)));


extern volatile __bit T3SYNC __attribute__((address(0x7D8A)));


extern volatile __bit TMR0IE __attribute__((address(0x7F95)));


extern volatile __bit TMR0IF __attribute__((address(0x7F92)));


extern volatile __bit TMR0IP __attribute__((address(0x7F8A)));


extern volatile __bit TMR0ON __attribute__((address(0x7EAF)));


extern volatile __bit TMR1CS __attribute__((address(0x7E69)));


extern volatile __bit TMR1IE __attribute__((address(0x7CE8)));


extern volatile __bit TMR1IF __attribute__((address(0x7CF0)));


extern volatile __bit TMR1IP __attribute__((address(0x7CF8)));


extern volatile __bit TMR1ON __attribute__((address(0x7E68)));


extern volatile __bit TMR2IE __attribute__((address(0x7CE9)));


extern volatile __bit TMR2IF __attribute__((address(0x7CF1)));


extern volatile __bit TMR2IP __attribute__((address(0x7CF9)));


extern volatile __bit TMR2ON __attribute__((address(0x7E52)));


extern volatile __bit TMR3CS __attribute__((address(0x7D89)));


extern volatile __bit TMR3IE __attribute__((address(0x7D01)));


extern volatile __bit TMR3IF __attribute__((address(0x7D09)));


extern volatile __bit TMR3IP __attribute__((address(0x7D11)));


extern volatile __bit TMR3ON __attribute__((address(0x7D88)));


extern volatile __bit TO __attribute__((address(0x7E83)));


extern volatile __bit TOUTPS0 __attribute__((address(0x7E53)));


extern volatile __bit TOUTPS1 __attribute__((address(0x7E54)));


extern volatile __bit TOUTPS2 __attribute__((address(0x7E55)));


extern volatile __bit TOUTPS3 __attribute__((address(0x7E56)));


extern volatile __bit TRISA0 __attribute__((address(0x7C90)));


extern volatile __bit TRISA1 __attribute__((address(0x7C91)));


extern volatile __bit TRISA2 __attribute__((address(0x7C92)));


extern volatile __bit TRISA3 __attribute__((address(0x7C93)));


extern volatile __bit TRISA4 __attribute__((address(0x7C94)));


extern volatile __bit TRISA5 __attribute__((address(0x7C95)));


extern volatile __bit TRISA6 __attribute__((address(0x7C96)));


extern volatile __bit TRISB0 __attribute__((address(0x7C98)));


extern volatile __bit TRISB1 __attribute__((address(0x7C99)));


extern volatile __bit TRISB2 __attribute__((address(0x7C9A)));


extern volatile __bit TRISB3 __attribute__((address(0x7C9B)));


extern volatile __bit TRISB4 __attribute__((address(0x7C9C)));


extern volatile __bit TRISB5 __attribute__((address(0x7C9D)));


extern volatile __bit TRISB6 __attribute__((address(0x7C9E)));


extern volatile __bit TRISB7 __attribute__((address(0x7C9F)));


extern volatile __bit TRISC0 __attribute__((address(0x7CA0)));


extern volatile __bit TRISC1 __attribute__((address(0x7CA1)));


extern volatile __bit TRISC2 __attribute__((address(0x7CA2)));


extern volatile __bit TRISC3 __attribute__((address(0x7CA3)));


extern volatile __bit TRISC4 __attribute__((address(0x7CA4)));


extern volatile __bit TRISC5 __attribute__((address(0x7CA5)));


extern volatile __bit TRISC6 __attribute__((address(0x7CA6)));


extern volatile __bit TRISC7 __attribute__((address(0x7CA7)));


extern volatile __bit TRISD0 __attribute__((address(0x7CA8)));


extern volatile __bit TRISD1 __attribute__((address(0x7CA9)));


extern volatile __bit TRISD2 __attribute__((address(0x7CAA)));


extern volatile __bit TRISD3 __attribute__((address(0x7CAB)));


extern volatile __bit TRISD4 __attribute__((address(0x7CAC)));


extern volatile __bit TRISD5 __attribute__((address(0x7CAD)));


extern volatile __bit TRISD6 __attribute__((address(0x7CAE)));


extern volatile __bit TRISD7 __attribute__((address(0x7CAF)));


extern volatile __bit TRISE0 __attribute__((address(0x7CB0)));


extern volatile __bit TRISE1 __attribute__((address(0x7CB1)));


extern volatile __bit TRISE2 __attribute__((address(0x7CB2)));


extern volatile __bit TRMT __attribute__((address(0x7D61)));


extern volatile __bit TRMT1 __attribute__((address(0x7D61)));


extern volatile __bit TX __attribute__((address(0x7C16)));


extern volatile __bit TX1IE __attribute__((address(0x7CEC)));


extern volatile __bit TX1IF __attribute__((address(0x7CF4)));


extern volatile __bit TX1IP __attribute__((address(0x7CFC)));


extern volatile __bit TX8_9 __attribute__((address(0x7D66)));


extern volatile __bit TX9 __attribute__((address(0x7D66)));


extern volatile __bit TX91 __attribute__((address(0x7D66)));


extern volatile __bit TX9D __attribute__((address(0x7D60)));


extern volatile __bit TX9D1 __attribute__((address(0x7D60)));


extern volatile __bit TXD8 __attribute__((address(0x7D60)));


extern volatile __bit TXEN __attribute__((address(0x7D65)));


extern volatile __bit TXEN1 __attribute__((address(0x7D65)));


extern volatile __bit TXIE __attribute__((address(0x7CEC)));


extern volatile __bit TXIF __attribute__((address(0x7CF4)));


extern volatile __bit TXIP __attribute__((address(0x7CFC)));


extern volatile __bit UA __attribute__((address(0x7E39)));


extern volatile __bit ULPWUIN __attribute__((address(0x7C00)));


extern volatile __bit VREFM __attribute__((address(0x7C02)));


extern volatile __bit VREFP __attribute__((address(0x7C03)));


extern volatile __bit WAIT0 __attribute__((address(0x7E5C)));


extern volatile __bit WAIT1 __attribute__((address(0x7E5D)));


extern volatile __bit WCOL __attribute__((address(0x7E37)));


extern volatile __bit WM0 __attribute__((address(0x7E58)));


extern volatile __bit WM1 __attribute__((address(0x7E59)));


extern volatile __bit __attribute__((__deprecated__)) WR __attribute__((address(0x7D31)));


extern volatile __bit WRE __attribute__((address(0x7C21)));


extern volatile __bit WREN __attribute__((address(0x7D32)));


extern volatile __bit WRERR __attribute__((address(0x7D33)));


extern volatile __bit ZERO __attribute__((address(0x7EC2)));


extern volatile __bit nA __attribute__((address(0x7E3D)));


extern volatile __bit nADDRESS __attribute__((address(0x7E3D)));


extern volatile __bit nBOR __attribute__((address(0x7E80)));


extern volatile __bit nDONE __attribute__((address(0x7E12)));


extern volatile __bit nIPEN __attribute__((address(0x7E87)));


extern volatile __bit nPD __attribute__((address(0x7E82)));


extern volatile __bit nPOR __attribute__((address(0x7E81)));


extern volatile __bit nRBPU __attribute__((address(0x7F8F)));


extern volatile __bit nRC8 __attribute__((address(0x7D5E)));


extern volatile __bit nRI __attribute__((address(0x7E84)));


extern volatile __bit nT1SYNC __attribute__((address(0x7E6A)));


extern volatile __bit nT3SYNC __attribute__((address(0x7D8A)));


extern volatile __bit nTO __attribute__((address(0x7E83)));


extern volatile __bit nTX8 __attribute__((address(0x7D66)));


extern volatile __bit nW __attribute__((address(0x7E3A)));


extern volatile __bit nWRITE __attribute__((address(0x7E3A)));
# 315 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/pic18_chip_select.h" 2 3
# 9 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/pic18.h" 2 3
# 18 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/pic18.h" 3
__attribute__((__unsupported__("The " "flash_write" " routine is no longer supported. Please use the MPLAB X MCC."))) void flash_write(const unsigned char *, unsigned int, __far unsigned char *);
__attribute__((__unsupported__("The " "EraseFlash" " routine is no longer supported. Please use the MPLAB X MCC."))) void EraseFlash(unsigned long startaddr, unsigned long endaddr);







# 1 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/errata.h" 1 3
# 27 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/pic18.h" 2 3
# 49 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/pic18.h" 3
#pragma intrinsic(__nop)
extern void __nop(void);
# 158 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/pic18.h" 3
__attribute__((__unsupported__("The " "Read_b_eep" " routine is no longer supported. Please use the MPLAB X MCC."))) unsigned char Read_b_eep(unsigned int badd);
__attribute__((__unsupported__("The " "Busy_eep" " routine is no longer supported. Please use the MPLAB X MCC."))) void Busy_eep(void);
__attribute__((__unsupported__("The " "Write_b_eep" " routine is no longer supported. Please use the MPLAB X MCC."))) void Write_b_eep(unsigned int badd, unsigned char bdat);
# 194 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/pic18.h" 3
unsigned char __t1rd16on(void);
unsigned char __t3rd16on(void);






#pragma intrinsic(_delay)
extern __attribute__((nonreentrant)) void _delay(unsigned long);
#pragma intrinsic(_delaywdt)
extern __attribute__((nonreentrant)) void _delaywdt(unsigned long);
#pragma intrinsic(_delay3)
extern __attribute__((nonreentrant)) void _delay3(unsigned char);
# 33 "/opt/microchip/mplabx/v5.45/packs/Microchip/PIC18Fxxxx_DFP/1.2.26/xc8/pic/include/xc.h" 2 3
# 78 "main.c" 2

# 1 "./init_PIC.h" 1
# 77 "./init_PIC.h"
void init_PORTS(void);
void init_interrupts(void);
# 79 "main.c" 2

# 1 "./serial_rs232.h" 1
# 80 "./serial_rs232.h"
void init_USART(void);
void serial_tx_char(unsigned char val);
# 80 "main.c" 2




static double const Fosc = 4000000;
static double Tosc = 1/Fosc;
static double const desired_BaudRate = 9600;
static _Bool New_char_RX = 0;
_Bool stop = 0;
int lenth_of_array = 30;
char testarray [30];
char sequence[]="CAB";
unsigned char State='0';
int displacement_X, displacement_Y, rotation;

int X_Pick, X_Place, Y_Pick, Y_Place, Pick_Angle, Place_Angle, X_diff=0, Y_diff=0, Angle_diff=0;
char X_dir, Y_dir, Rot_dir;
unsigned char RX_Char;
volatile unsigned char input_str[10]=" ";
# 132 "main.c"
int X_Pick_D;
int Y_Pick_D;
int Pick_Angle_D;

int X_Place_D;
int Y_Place_D;
int Place_Angle_D;

int X_diff;
int Y_diff;
int angle_diff;
# 163 "main.c"
void system_init (void);
void X_axis (char direction);
void Y_axis (char direction);
void Z_axis (char direction);
void Twister (char direction);
void Tweezer (void);
void ms_delay(unsigned int val);
int pick_and_place(char Componnt);
int fetch_parameters(char Componnt);
void return_to_initial(void);
void __attribute__((picinterrupt(("")))) Rx_char_USART (void);
void Z_axis_and_Tweezer(void);

void print_string(char strng[]);
void start_up_menu(void);
void change_sequence(void);
void add_component(void);
void remove_component(void);







void main(void)
{
    unsigned char RX_Char = ' ';

    init_PORTS();
    init_USART();
    init_interrupts();

    (INTCONbits.GIE = 1);
    while(1){

        start_up_menu();
    }
}





void start_up_menu(void){

    print_string("\n\n1. Start Sequence\n2. Change sequence\n3. Add new component\n4. Remove a component");

    while(1){
        if(New_char_RX){
            New_char_RX=0;
            switch(State){
                case '1':
                    print_string("\nStarting sequence:");
                    print_string(sequence);
                    print_string("\nEnter 'Q' to exit to MAIN MENU");
                    while(!New_char_RX){
                        int i =0;
                        do{
                            pick_and_place(sequence[i]);
                        }while(sequence[i++]!='\n'&& !stop);
                    }
                    break;

                case '2':
                    change_sequence();
                    return_to_initial();
                    break;

                case '3':
                    add_component();
                    return_to_initial();
                    break;

                case '4':
                    remove_component();
                    return_to_initial();
                    break;

                case 'Q':
                    New_char_RX=0;
                    stop=0;
                    break;
            }
            break;
        }
    }
}

void change_sequence(void){
    print_string("\nThe current sequence is: ");
    print_string(sequence);
    print_string("\nPlease give the new sequence as input\r");
    int i = 0;
    while(1) {
        if(New_char_RX){
            do{
                sequence[i]=input_str[i];
            }while(input_str[i++]!='\n');
            New_char_RX=0;
            break;
        }
    }
}

void add_component(void){
    char add_componnt[1]=" ";
    print_string("\nThe Current sequence is:");
    print_string(sequence);
    print_string("\nEnter the component you want to add: A, B, C or D?");
    while(1){
        if(New_char_RX){
            add_componnt[0]=input_str[0];
            if(add_componnt[0]=='D'){
                New_char_RX=0;
                print_string("\nEnter the pick position X:");
                while(1){
                    if(New_char_RX){
                        X_Pick_D=atoi(input_str);
                        New_char_RX=0;
                        break;
                    }
                }
                print_string("\nEnter the pick position Y:");
                while(1){
                    if(New_char_RX){
                        Y_Pick_D=(atoi(input_str));
                        New_char_RX=0;
                        break;
                    }
                }
                print_string("\nEnter the pick orientation:");
                while(1){
                    if(New_char_RX){
                        Pick_Angle_D=(atoi(input_str));
                        New_char_RX=0;
                        break;
                    }
                }
                print_string("\nEnter the place position X:");
                while(1){
                    if(New_char_RX){
                        X_Place_D=(atoi(input_str));
                        New_char_RX=0;
                        break;
                    }
                }
                print_string("\nEnter the place position Y:");
                while(1){
                    if(New_char_RX){
                        Y_Place_D=(atoi(input_str));
                        New_char_RX=0;
                        break;
                    }
                }
                print_string("\nEnter the place orientation:");
                while(1){
                        if(New_char_RX){
                        Place_Angle_D=(atoi(input_str));
                        New_char_RX=0;
                        break;
                    }
                }
                break;
            }
            else{
                New_char_RX=0;
                break;
            }
        }
    }
    State='0';
    strncat(sequence,add_componnt,1);
    print_string("\n\nComponent Added: ");
    serial_tx_char(add_componnt[0]);
    New_char_RX=0;
}

void remove_component(void){
    print_string("\nWhich component would you like to remove from the  sequence?\n");
    print_string(sequence);
    while(1){
        if(New_char_RX){
            New_char_RX=0;
            char rm_comp = input_str[0];
            char *ptr;
            int indxToDel;
            ptr=strrchr(sequence,rm_comp);

            if (rm_comp=='Q'){
                break;
            }
            else if(ptr != ((void*)0)){
                indxToDel = ptr - sequence;
                memmove(&sequence[indxToDel], &sequence[indxToDel + 1], strlen(sequence) - indxToDel);
                print_string("\ncomponent removed: ");
                serial_tx_char(rm_comp);
                stop=0;
                break;
            }

            else{
                print_string("\ncomponent is not in the sequence. Try selecting another component: \n");
                while(!New_char_RX){}
            }
        }
    }
    New_char_RX=0;
}



void print_string(char strng[])
{
    for(int i =0; strng[i]!=((void*)0); i++){
        serial_tx_char(strng[i]);
    }
}


void X_axis (char direction){
    if(!New_char_RX){
    if (direction == 0){
        PORTB = 0b00000011;
        ms_delay(1);
        PORTB = 0b00000110;
        ms_delay(1);
        PORTB = 0b00001100;
        ms_delay(1);
        PORTB = 0b00001001;
        ms_delay(1);
        PORTB = 0b00000011;
        ms_delay(1);
    }
    if (direction == 1){
        PORTB = 0b00001001;
        ms_delay(1);
        PORTB = 0b00001100;
        ms_delay(1);
        PORTB = 0b00000110;
        ms_delay(1);
        PORTB = 0b00000011;
        ms_delay(1);
        PORTB = 0b00001001;
        ms_delay(1);
    }}
    else{
        stop=1;
    }
}

void Y_axis (char direction){
    if(!New_char_RX){
    if (direction == 0){
        PORTB = 0b00110000;
        ms_delay(1);
        PORTB = 0b01100000;
        ms_delay(1);
        PORTB = 0b11000000;
        ms_delay(1);
        PORTB = 0b10010000;
        ms_delay(1);
        PORTB = 0b00110000;
        ms_delay(1);
    }
    if (direction == 1){
        PORTB = 0b10010000;
        ms_delay(1);
        PORTB = 0b11000000;
        ms_delay(1);
        PORTB = 0b01100000;
        ms_delay(1);
        PORTB = 0b00110000;
        ms_delay(1);
        PORTB = 0b10010000;
        ms_delay(1);
    } }
    else{
        stop=1;
    }
}

void Z_axis (char direction){
    if(!New_char_RX){
    if (direction == 0){
        PORTD = 0b00000011;
        ms_delay(1);
        PORTD = 0b00000110;
        ms_delay(1);
        PORTD = 0b00001100;
        ms_delay(1);
        PORTD = 0b00001001;
        ms_delay(1);
        PORTD = 0b00000011;
        ms_delay(1);
    }
    if (direction == 1){
        PORTD = 0b00001001;
        ms_delay(1);
        PORTD = 0b00001100;
        ms_delay(1);
        PORTD = 0b00000110;
        ms_delay(1);
        PORTD = 0b00000011;
        ms_delay(1);
        PORTD = 0b00001001;
        ms_delay(1);
    }}
    else{
        stop=1;
    }
}

void Twister (char direction){
    if(!New_char_RX){
    if (direction == 0){
        PORTD = 0b00110000;
        ms_delay(1);
        PORTD = 0b11000000;
        ms_delay(1);
        PORTD = 0b10010000;
        ms_delay(1);
        PORTD = 0b00110000;
        ms_delay(1);
    }
    if (direction == 1){
        PORTD = 0b10010000;
        ms_delay(1);
        PORTD = 0b01100000;
        ms_delay(1);
        PORTD = 0b00110000;
        ms_delay(1);
        PORTD = 0b10010000;
        ms_delay(1);
    }
    }
    else{
        stop=1;
    }
}


void ms_delay(unsigned int val)
{
     unsigned int i,j;
        for(i=0;i<val;i++)
            for(j=0;j<1650;j++);
}
int fetch_parameters(char Componnt){
    if(Componnt=='A')
        {
            X_Pick=3;
            Y_Pick=4;
            Pick_Angle=0;

            X_Place=17;
            Y_Place=5;
            Place_Angle=270;
        }
        else if(Componnt=='B'){
            X_Pick=3;
            Y_Pick=10;
            Pick_Angle=90;

            X_Place=10;
             Y_Place=5;
            Place_Angle=180;
        }
        else if(Componnt=='C'){
            X_Pick=3;
            Y_Pick=15;
            Pick_Angle=90;

            X_Place=16;
            Y_Place=14;
            Place_Angle=270;
        }
        else if(Componnt=='D'){
            X_Pick=X_Pick_D;
            Y_Pick=Y_Pick_D;
            Pick_Angle=Pick_Angle_D;

            X_Place=X_Place_D;
            Y_Place=Y_Place_D;
            Place_Angle=Place_Angle_D;
        }

}

int pick_and_place(char Componnt)
{


    fetch_parameters(Componnt);

        X_dir=((X_diff<X_Pick)? 1:0);
        Y_dir=((Y_diff<Y_Pick)? 1:0);
        Rot_dir=((Angle_diff<Pick_Angle)? 1:0);

        for(int i = 0; (i<(((X_Pick-X_diff) > 0 ? (X_Pick-X_diff) : -(X_Pick-X_diff)))) && !stop; i++){X_axis(X_dir);}
        for(int i = 0; (i<(((Y_Pick-Y_diff) > 0 ? (Y_Pick-Y_diff) : -(Y_Pick-Y_diff)))) && !stop; i++){Y_axis(Y_dir);}
        for(int i = 0; (i<(((Angle_diff-Pick_Angle) > 0 ? (Angle_diff-Pick_Angle) : -(Angle_diff-Pick_Angle))/3.6)) && !stop; i++){Twister(Rot_dir);}

        for(int i=0; i<3; i++){Z_axis(1);}


        for(int i=0; i<3; i++){Z_axis(0);}



        X_dir=((X_Pick<X_Place)? 1:0);
        Y_dir=((Y_Pick<Y_Place)? 1:0);
        Rot_dir=((Pick_Angle<Place_Angle)? 1:0);

        for(int i = 0; (i<(((X_Place-X_diff) > 0 ? (X_Place-X_diff) : -(X_Place-X_diff)))) && !stop; i++){X_axis(X_dir);}
        for(int i = 0; (i<(((Y_Place-Y_diff) > 0 ? (Y_Place-Y_diff) : -(Y_Place-Y_diff)))) && !stop; i++){Y_axis(Y_dir);}
        for(int i = 0; (i<(((Pick_Angle-Place_Angle) > 0 ? (Pick_Angle-Place_Angle) : -(Pick_Angle-Place_Angle))/3.6)) && !stop; i++){Twister(Rot_dir);}
        for(int i=0; i<3; i++){Z_axis(1);}


        for(int i=0; i<3; i++){Z_axis(0);}

        Angle_diff= Place_Angle;
        X_diff = X_Place;
        Y_diff = Y_Place;


}

void return_to_initial(void){

    for(int i = 0; i<X_diff; i++){X_axis(1);}
    for(int i = 0; i<Y_diff; i++){Y_axis(1);}
    for(int i = 0; i<Angle_diff; i++){Twister(1);}
    for(int i=0; i<3 && !stop; i++){Z_axis(0);}

}
# 611 "main.c"
void Z_axis_and_Tweezer(){
     for(int i = 0;i<15; i++){Z_axis(0);}

     for(int i = 0; i<15; i++){Z_axis(1);}
 }

void __attribute__((picinterrupt(("")))) Rx_char_USART(void)
{
    int i=0;
    do
    {
        while(!RCIF){};
        input_str[i]=RCREG;
    }while(input_str[i++] != '\n');

    PIR1bits.RCIF = 0;
    New_char_RX = 1;
    State = input_str[0];
}
