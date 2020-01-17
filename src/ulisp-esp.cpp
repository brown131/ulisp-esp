/* uLisp ESP Version 3.0b - www.ulisp.com
   David Johnson-Davies - www.technoblogy.com - 11th January 2020

   Licensed under the MIT license: https://opensource.org/licenses/MIT
*/

// Lisp Library
//const char LispLibrary[] = "";

// Compile options

// #define resetautorun
#define printfreespace
#define serialmonitor
// #define printgcs
// #define sdcardsupport
#define lisplibrary

// Includes

#include <setjmp.h>
#include <SPI.h>
#include <Wire.h>
#include <limits.h>
#include <EEPROM.h>
#include <alloc/spiram_alloc.h>
#if defined (ESP8266)
  //#include <ESP8266WiFi.h>
  #include <FS.h>
#elif defined (ESP32)
  #include <WiFi.h>
  #include <SPIFFS.h>
#endif
#include "LispLibrary.h"

#if defined(sdcardsupport)
  #include <SD.h>
  #define SDSIZE 172
#else
  #define SDSIZE 0
#endif

using namespace virtmem;

// C Macros

#define nil                Null
#define car(x)             (((VPtr<object, SPIRAMVAlloc>) (x))->ptr.car)
#define cdr(x)             (((VPtr<object, SPIRAMVAlloc>) (x))->ptr.cdr)

#define first(x)           (((VPtr<object, SPIRAMVAlloc>) (x))->ptr.car)
#define second(x)          (car(cdr(x)))
#define cddr(x)            (cdr(cdr(x)))
#define third(x)           (car(cdr(cdr(x))))

#define push(x, y)         ((y) = cons((x),(y)))
#define pop(y)             ((y) = cdr(y))

#define integerp(x)        ((x) != nil && (x)->val.type == NUMBER)
#define floatp(x)          ((x) != nil && (x)->val.type == FLOAT)
#define symbolp(x)         ((x) != nil && (x)->val.type == SYMBOL)
#define stringp(x)         ((x) != nil && (x)->val.type == STRING)
#define characterp(x)      ((x) != nil && (x)->val.type == CHARACTER)
#define streamp(x)         ((x) != nil && (x)->val.type == STREAM)

#define mark(x)            (car(x).setRawNum(car(x).getRawNum() | MARKBIT))
#define unmark(x)          (car(x).setRawNum(car(x).getRawNum() & ~MARKBIT))
#define marked(x)          ((car(x).getRawNum() & MARKBIT) != 0)
//((((uintptr_t)(car(x))) & MARKBIT) != 0)
#define MARKBIT            1

#define setflag(x)         (Flags = Flags | 1<<(x))
#define clrflag(x)         (Flags = Flags & ~(1<<(x)))
#define tstflag(x)         (Flags & 1<<(x))

// Constants

const int TRACEMAX = 3; // Number of traced functions
enum type { ZERO=0, SYMBOL=2, NUMBER=4, STREAM=6, CHARACTER=8, FLOAT=10, STRING=12, PAIR=14 };  // STRING and PAIR must be last
enum token { UNUSED, BRA, KET, QUO, DOT };
enum stream { SERIALSTREAM, I2CSTREAM, SPISTREAM, SDSTREAM, SPIFFSSTREAM, WIFISTREAM };

enum function { NIL, TEE, NOTHING, OPTIONAL, AMPREST, LAMBDA, LET, LETSTAR, CLOSURE, SPECIAL_FORMS, QUOTE,
DEFUN, DEFVAR, SETQ, LOOP, RETURN, PUSH, POP, INCF, DECF, SETF, DOLIST, DOTIMES, TRACE, UNTRACE,
FORMILLIS, WITHSERIAL, WITHI2C, WITHSPI, WITHSDCARD, WITHSPIFFS, WITHCLIENT, TAIL_FORMS, PROGN, IF, COND, WHEN,
UNLESS, CASE, AND, OR, FUNCTIONS, NOT, nilFN, CONS, ATOM, LISTP, CONSP, SYMBOLP, STREAMP, EQ, CAR, FIRST,
CDR, REST, CAAR, CADR, SECOND, CDAR, CDDR, CAAAR, CAADR, CADAR, CADDR, THIRD, CDAAR, CDADR, CDDAR, CDDDR,
LENGTH, LIST, REVERSE, NTH, ASSOC, MEMBER, APPLY, FUNCALL, APPEND, MAPC, MAPCAR, MAPCAN, ADD, SUBTRACT,
MULTIPLY, DIVIDE, MOD, ONEPLUS, ONEMINUS, ABS, RANDOM, MAXFN, MINFN, NOTEQ, NUMEQ, LESS, LESSEQ, GREATER,
GREATEREQ, PLUSP, MINUSP, ZEROP, ODDP, EVENP, INTEGERP, NUMBERP, FLOATFN, FLOATP, SIN, COS, TAN, ASIN,
ACOS, ATAN, SINH, COSH, TANH, EXP, SQRT, LOG, EXPT, CEILING, FLOOR, TRUNCATE, ROUND, CHAR, CHARCODE,
CODECHAR, CHARACTERP, STRINGP, STRINGEQ, STRINGLESS, STRINGGREATER, SORT, STRINGFN, CONCATENATE, SUBSEQ,
READFROMSTRING, PRINCTOSTRING, PRIN1TOSTRING, LOGAND, LOGIOR, LOGXOR, LOGNOT, ASH, LOGBITP, EVAL, GLOBALS,
LOCALS, MAKUNBOUND, BREAK, READ, PRIN1, PRINT, PRINC, TERPRI, READBYTE, READLINE, WRITEBYTE, WRITESTRING,
WRITELINE, RESTARTI2C, GC, ROOM, SAVEIMAGE, LOADIMAGE, CLS, PINMODE, DIGITALREAD, DIGITALWRITE,
ANALOGREAD, ANALOGWRITE, DELAY, MILLIS, SLEEP, NOTE, EDIT, PPRINT, PPRINTALL, REQUIRE, LISTLIBRARY,
AVAILABLE, WIFISERVER, WIFISOFTAP, CONNECTED, WIFILOCALIP, WIFICONNECT, ENDFUNCTIONS };

// Typedefs

typedef unsigned int symbol_t;

typedef struct sobject {
  union {
    struct {
      VPtr<sobject, SPIRAMVAlloc> car;
      VPtr<sobject, SPIRAMVAlloc> cdr;
    } ptr;
    struct {
      unsigned int type;
      union {
        symbol_t name;
        int integer;
        float single_float;
      };
    } val;
  };
} object;

typedef VPtr<object, SPIRAMVAlloc>(*fn_ptr_type)(VPtr<object, SPIRAMVAlloc>, VPtr<object, SPIRAMVAlloc>);

typedef struct {
  const char *string;
  fn_ptr_type fptr;
  uint8_t min;
  uint8_t max;
} tbl_entry_t;

typedef int (*gfun_t)();
typedef void (*pfun_t)(char);
typedef int PinMode;

// Workspace
#define WORDALIGNED __attribute__((aligned (4)))
#define BUFFERSIZE 34  // Number of bits+2

#if defined(ESP8266)
  #define WORKSPACESIZE 3072-SDSIZE       /* Cells (8*bytes) */
  #define EEPROMSIZE 4096                 /* Bytes available for EEPROM */
  #define SYMBOLTABLESIZE 512             /* Bytes */
  #define SDCARD_SS_PIN 10
  uint8_t _end;
  typedef int BitOrder;

#elif defined(ESP32)
  #define WORKSPACESIZE 8000-SDSIZE       /* Cells (8*bytes) */
  #define EEPROMSIZE 4096                 /* Bytes available for EEPROM */
  #define SYMBOLTABLESIZE 1024            /* Bytes */
  #define analogWrite(x,y) dacWrite((x),(y))
  #define SDCARD_SS_PIN 13
  uint8_t _end;
  typedef int BitOrder;

#endif

VPtr<object, SPIRAMVAlloc> Workspace[WORKSPACESIZE] WORDALIGNED;
char SymbolTable[SYMBOLTABLESIZE];

// Global variables

SPIRAMVAlloc valloc;
VPtr<object, SPIRAMVAlloc> Null = VPtr<object, SPIRAMVAlloc> { };
jmp_buf exception;
unsigned int Freespace = 0;
VPtr<object, SPIRAMVAlloc> Freelist;
char *SymbolTop = SymbolTable;
unsigned int I2CCount;
unsigned int TraceFn[TRACEMAX];
unsigned int TraceDepth[TRACEMAX];

VPtr<object, SPIRAMVAlloc> GlobalEnv;
VPtr<object, SPIRAMVAlloc> GCStack;
VPtr<object, SPIRAMVAlloc> GlobalString;
int GlobalStringIndex = 0;
char BreakLevel = 0;
char LastChar = 0;
char LastPrint = 0;

VPtr<object, SPIRAMVAlloc> BRAToken = VPtr<object, SPIRAMVAlloc>();
VPtr<object, SPIRAMVAlloc> KETToken = VPtr<object, SPIRAMVAlloc>();
VPtr<object, SPIRAMVAlloc> QUOToken = VPtr<object, SPIRAMVAlloc>();
VPtr<object, SPIRAMVAlloc> DOTToken = VPtr<object, SPIRAMVAlloc>();

// Flags
enum flag { PRINTREADABLY, RETURNFLAG, ESCAPE, EXITEDITOR, LIBRARYLOADED, NOESC };
volatile char Flags = 0b00001; // PRINTREADABLY set by default

// Forward references
VPtr<object, SPIRAMVAlloc> tee;
VPtr<object, SPIRAMVAlloc> tf_progn (VPtr<object, SPIRAMVAlloc> form, VPtr<object, SPIRAMVAlloc> env);
VPtr<object, SPIRAMVAlloc> eval (VPtr<object, SPIRAMVAlloc> form, VPtr<object, SPIRAMVAlloc> env);
VPtr<object, SPIRAMVAlloc> read ();
void repl(VPtr<object, SPIRAMVAlloc> env);
void printobject (VPtr<object, SPIRAMVAlloc> form, pfun_t pfun);
char *lookupbuiltin (symbol_t name);
intptr_t lookupfn (symbol_t name);
int builtin (char* n);
void error (symbol_t fname, PGM_P string, VPtr<object, SPIRAMVAlloc> symbol);
void error2 (symbol_t fname, PGM_P string);

inline int maxbuffer (char *buffer);
char nthchar (VPtr<object, SPIRAMVAlloc> string, int n);
boolean listp (VPtr<object, SPIRAMVAlloc> x);
VPtr<object, SPIRAMVAlloc> apply (symbol_t name, VPtr<object, SPIRAMVAlloc> function, VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env);
void pserial (char c);
void pfl (pfun_t pfun);
void pfstring (const char *s, pfun_t pfun);
char *symbolname (symbol_t x);
void pln (pfun_t pfun);
void pstring (char *s, pfun_t pfun);
char *lookupsymbol (symbol_t name) ;
int listlength (symbol_t name, VPtr<object, SPIRAMVAlloc> list);
uint8_t lookupmin (symbol_t name);
uint8_t lookupmax (symbol_t name);
char *cstring (VPtr<object, SPIRAMVAlloc> form, char *buffer, int buflen);
void pint (int i, pfun_t pfun);
void testescape ();
int gserial ();
VPtr<object, SPIRAMVAlloc> read (gfun_t gfun);
void deletesymbol (symbol_t name);
void printstring (VPtr<object, SPIRAMVAlloc> form, pfun_t pfun);
VPtr<object, SPIRAMVAlloc> edit (VPtr<object, SPIRAMVAlloc> fun);
void superprint (VPtr<object, SPIRAMVAlloc> form, int lm, pfun_t pfun);
void supersub (VPtr<object, SPIRAMVAlloc> form, int lm, int super, pfun_t pfun);
int subwidthlist (VPtr<object, SPIRAMVAlloc> form, int w);
int glibrary ();

// Set up workspace

void initworkspace () {
  Freelist = nil;
  for (int i=WORKSPACESIZE-1; i>=0; i--) {
    Workspace[i] = valloc.alloc<object>();
    object obj = *Workspace[i];
    obj.ptr.car = nil;
    obj.ptr.cdr = Freelist;
    Freelist = Workspace[i];
    Freespace++;
  }
}

VPtr<object, SPIRAMVAlloc> myalloc () {
  if (Freespace == 0) error2(0, PSTR("no room"));
  VPtr<object, SPIRAMVAlloc> temp = Freelist;
  Freelist = cdr(Freelist);
  Freespace--;
  return temp;
}

inline void myfree (VPtr<object, SPIRAMVAlloc> obj) {
  car(obj) = nil;
  cdr(obj) = Freelist;
  Freelist = obj;
  Freespace++;
}

// Make each type of object

VPtr<object, SPIRAMVAlloc> number (int n) {
  VPtr<object, SPIRAMVAlloc> ptr = myalloc();
  ptr->val.type = NUMBER;
  ptr->val.integer = n;
  return ptr;
}

VPtr<object, SPIRAMVAlloc> makefloat (float f) {
  VPtr<object, SPIRAMVAlloc> ptr = myalloc();
  ptr->val.type = FLOAT;
  ptr->val.single_float = f;
  return ptr;
}

VPtr<object, SPIRAMVAlloc> character (char c) {
  VPtr<object, SPIRAMVAlloc> ptr = myalloc();
  ptr->val.type = CHARACTER;
  ptr->val.integer = c;
  return ptr;
}

VPtr<object, SPIRAMVAlloc> cons (VPtr<object, SPIRAMVAlloc> arg1, VPtr<object, SPIRAMVAlloc> arg2) {
  VPtr<object, SPIRAMVAlloc> ptr = myalloc();
  ptr->ptr.car = arg1;
  ptr->ptr.cdr = arg2;
  return ptr;
}

VPtr<object, SPIRAMVAlloc> symbol (symbol_t name) {
  VPtr<object, SPIRAMVAlloc> ptr = myalloc();
  ptr->val.type = SYMBOL;
  ptr->val.name = name;
  return ptr;
}

VPtr<object, SPIRAMVAlloc> newsymbol (symbol_t name) {
  for (int i=WORKSPACESIZE-1; i>=0; i--) {
    VPtr<object, SPIRAMVAlloc> obj = Workspace[i];
    if (obj->val.type == SYMBOL && obj->val.name == name) return obj;
  }
  return symbol(name);
}

VPtr<object, SPIRAMVAlloc> stream (unsigned char streamtype, unsigned char address) {
  VPtr<object, SPIRAMVAlloc> ptr = myalloc();
  ptr->val.type = STREAM;
  ptr->val.integer = streamtype<<8 | address;
  return ptr;
}

// Garbage collection

void markobject (VPtr<object, SPIRAMVAlloc> obj) {
  MARK:
  if (obj == nil) return;
  if (marked(obj)) return;

  VPtr<object, SPIRAMVAlloc> arg = car(obj);
  unsigned int type = obj->val.type;
  mark(obj);
  
  if (type >= PAIR || type == ZERO) { // cons
    markobject(arg);
    obj = cdr(obj);
    goto MARK;
  }

  if (type == STRING) {
    obj = cdr(obj);
    while (obj != nil) {
      arg = car(obj);

      mark(obj);
      obj = arg;
    }
  }
}

void sweep () {
  Freelist = nil;
  Freespace = 0;
  for (int i=WORKSPACESIZE-1; i>=0; i--) {
    VPtr<object, SPIRAMVAlloc> obj = Workspace[i];
    if (!marked(obj)) myfree(obj); else unmark(obj);
  }
}

void gc (VPtr<object, SPIRAMVAlloc> form, VPtr<object, SPIRAMVAlloc> env) {
  #if defined(printgcs)
  int start = Freespace;
  #endif
  markobject(tee);
  markobject(GlobalEnv);
  markobject(GCStack);
  markobject(form);
  markobject(env);
  sweep();
  #if defined(printgcs)
  pfl(pserial); pserial('{'); pint(Freespace - start, pserial); pserial('}');
  #endif
}

// Compact image

void movepointer (VPtr<object, SPIRAMVAlloc> from, VPtr<object, SPIRAMVAlloc> to) {
  for (int i=0; i<WORKSPACESIZE; i++) {
    VPtr<object, SPIRAMVAlloc> obj = Workspace[i];
    unsigned int type = (obj->val.type) & ~MARKBIT;
    if (marked(obj) && (type >= STRING || type==ZERO)) {
      if (car(obj).getRawNum() == (from.getRawNum() | MARKBIT))
        car(obj).setRawNum(to.getRawNum() | MARKBIT);
      if (cdr(obj) == from) cdr(obj) = to;
    }
  }
  // Fix strings
  for (int i=0; i<WORKSPACESIZE; i++) {
    VPtr<object, SPIRAMVAlloc> obj = Workspace[i];
    if (marked(obj) && ((obj->val.type) & ~MARKBIT) == STRING) {
      obj = cdr(obj);
      while (obj != nil) {
        if (cdr(obj) == to) cdr(obj) = from;
        unmark(obj);
      }
    }
  }
}
  
int compactimage (VPtr<object, SPIRAMVAlloc> *arg) {
  markobject(tee);
  markobject(GlobalEnv);
  markobject(GCStack);
  VPtr<object, SPIRAMVAlloc> firstfree = Workspace[0];
  while (marked(firstfree)) firstfree++;
  VPtr<object, SPIRAMVAlloc> obj = Workspace[WORKSPACESIZE-1];
  while (firstfree < obj) {
    if (marked(obj)) {
      car(firstfree) = car(obj);
      cdr(firstfree) = cdr(obj);
      unmark(obj);
      movepointer(obj, firstfree);
      if (GlobalEnv == obj) GlobalEnv = firstfree;
      if (GCStack == obj) GCStack = firstfree;
      if (*arg == obj) *arg = firstfree;
      while (marked(firstfree)) firstfree++;
    }
    obj--;
  }
  sweep();
  return firstfree.getRawNum() - Workspace->getRawNum();
}

// Make SD card filename

char *MakeFilename (VPtr<object, SPIRAMVAlloc> arg) {
  char *buffer = SymbolTop;
  int max = maxbuffer(buffer);
  buffer[0]='/';
  int i = 1;
  do {
    char c = nthchar(arg, i-1);
    if (c == '\0') break;
    buffer[i++] = c;
  } while (i<max);
  buffer[i] = '\0';
  return buffer;
}

// Save-image and load-image

#if defined(sdcardsupport)
void SDWriteInt (File file, int data) {
  file.write(data & 0xFF); file.write(data>>8 & 0xFF);
  file.write(data>>16 & 0xFF); file.write(data>>24 & 0xFF);
}
#endif

void EpromWriteInt(int *addr, uintptr_t data) {
  EEPROM.write((*addr)++, data & 0xFF); EEPROM.write((*addr)++, data>>8 & 0xFF);
  EEPROM.write((*addr)++, data>>16 & 0xFF); EEPROM.write((*addr)++, data>>24 & 0xFF);
}

void SpiffsWriteInt (File file, int data) {
  file.write(data & 0xFF); file.write(data>>8 & 0xFF);
  file.write(data>>16 & 0xFF); file.write(data>>24 & 0xFF);
}

unsigned int saveimage (VPtr<object, SPIRAMVAlloc> arg) {
#if defined(sdcardsupport)
  unsigned int imagesize = compactimage(&arg);
  SD.begin(SDCARD_SS_PIN);
  File file;
  if (stringp(arg)) {
    file = SD.open(MakeFilename(arg), FILE_WRITE);
    arg = nil;
  } else if (arg == nil || listp(arg)) file = SD.open("/ULISP.IMG", FILE_WRITE);
  else error(SAVEIMAGE, PSTR("illegal argument"), arg);
  if (!file) error2(SAVEIMAGE, PSTR("problem saving to SD card"));
  SDWriteInt(file, (uintptr_t)arg);
  SDWriteInt(file, imagesize);
  SDWriteInt(file, (uintptr_t)GlobalEnv);
  SDWriteInt(file, (uintptr_t)GCStack);
  #if SYMBOLTABLESIZE > BUFFERSIZE
    SDWriteInt(file, (uintptr_t)SymbolTop);
    for (int i=0; i<SYMBOLTABLESIZE; i++) file.write(SymbolTable[i]);
  #endif
  for (unsigned int i=0; i<imagesize; i++) {
    VPtr<object, SPIRAMVAlloc> obj = &Workspace[i];
    SDWriteInt(file, (uintptr_t)car(obj));
    SDWriteInt(file, (uintptr_t)cdr(obj));
  }
  file.close();
  return imagesize;
#else
  unsigned int imagesize = compactimage(&arg);
  SPIFFS.begin();
  File file;
  if (stringp(arg)) {
    file = SPIFFS.open(MakeFilename(arg), "w");
    arg = nil;
  } else if (arg == nil || listp(arg)) file = SPIFFS.open("/ULISP.IMG", "w");
  else error(SAVEIMAGE, PSTR("illegal argument"), arg);
  if (!file) error2(SAVEIMAGE, PSTR("problem saving to SPIFFS"));
  SpiffsWriteInt(file, arg.getRawNum());
  SpiffsWriteInt(file, imagesize);
  SpiffsWriteInt(file, GlobalEnv.getRawNum());
  SpiffsWriteInt(file, GCStack.getRawNum());
  #if SYMBOLTABLESIZE > BUFFERSIZE
    SpiffsWriteInt(file, (uintptr_t)SymbolTop);
    for (int i=0; i<SYMBOLTABLESIZE; i++) file.write(SymbolTable[i]);
  #endif
  for (unsigned int i=0; i<imagesize; i++) {
    VPtr<object, SPIRAMVAlloc> obj = Workspace[i];
    SpiffsWriteInt(file, car(obj).getRawNum());
    SpiffsWriteInt(file, cdr(obj).getRawNum());
  }
  file.close();
  return imagesize;
#endif
}

#if defined(sdcardsupport)
int SDReadInt (File file) {
  uintptr_t b0 = file.read(); uintptr_t b1 = file.read();
  uintptr_t b2 = file.read(); uintptr_t b3 = file.read();
  return b0 | b1<<8 | b2<<16 | b3<<24;
}
#endif

int EpromReadInt (int *addr) {
  uint8_t b0 = EEPROM.read((*addr)++); uint8_t b1 = EEPROM.read((*addr)++);
  uint8_t b2 = EEPROM.read((*addr)++); uint8_t b3 = EEPROM.read((*addr)++);
  return b0 | b1<<8 | b2<<16 | b3<<24;
}

int SpiffsReadInt (File file) {
  uintptr_t b0 = file.read(); uintptr_t b1 = file.read();
  uintptr_t b2 = file.read(); uintptr_t b3 = file.read();
  return b0 | b1<<8 | b2<<16 | b3<<24;
}

unsigned int loadimage (VPtr<object, SPIRAMVAlloc> arg) {
#if defined(sdcardsupport)
  SD.begin(SDCARD_SS_PIN);
  File file;
  if (stringp(arg)) file = SD.open(MakeFilename(arg));
  else if (arg == nil) file = SD.open("/ULISP.IMG");
  else error(LOADIMAGE, PSTR("illegal argument"), arg);
  if (!file) error2(LOADIMAGE, PSTR("problem loading from SD card"));
  SDReadInt(file);
  int imagesize = SDReadInt(file);
  GlobalEnv = (VPtr<object, SPIRAMVAlloc>)SDReadInt(file);
  GCStack = (VPtr<object, SPIRAMVAlloc>)SDReadInt(file);
  #if SYMBOLTABLESIZE > BUFFERSIZE
  SymbolTop = (char *)SDReadInt(file);
  for (int i=0; i<SYMBOLTABLESIZE; i++) SymbolTable[i] = file.read();
  #endif
  for (int i=0; i<imagesize; i++) {
    VPtr<object, SPIRAMVAlloc> obj = &Workspace[i];
    car(obj) = (VPtr<object, SPIRAMVAlloc>)SDReadInt(file);
    cdr(obj) = (VPtr<object, SPIRAMVAlloc>)SDReadInt(file);
  }
  file.close();
  gc(nil, nil);
  return imagesize;
#else
  SPIFFS.begin();
  File file;
  if (stringp(arg)) file = SPIFFS.open(MakeFilename(arg), "r");
  else if (arg == nil) file = SPIFFS.open("/ULISP.IMG", "r");
  else error(LOADIMAGE, PSTR("illegal argument"), arg);
  if (!file) error2(LOADIMAGE, PSTR("problem loading from SPIFFS"));
  SpiffsReadInt(file);
  int imagesize = SpiffsReadInt(file);
  GlobalEnv.setRawNum(SpiffsReadInt(file));
  GCStack.setRawNum(SpiffsReadInt(file));
  #if SYMBOLTABLESIZE > BUFFERSIZE
  SymbolTop = (char *)SpiffsReadInt(file);
  for (int i=0; i<SYMBOLTABLESIZE; i++) SymbolTable[i] = file.read();
  #endif
  for (int i=0; i<imagesize; i++) {
    VPtr<object, SPIRAMVAlloc> obj = Workspace[i];
    car(obj).setRawNum(SpiffsReadInt(file));
    cdr(obj).setRawNum(SpiffsReadInt(file));
  }
  file.close();
  gc(nil, nil);
  return imagesize;
#endif
}

void autorunimage () {
#if defined(sdcardsupport)
  SD.begin(SDCARD_SS_PIN);
  File file = SD.open("/ULISP.IMG");
  if (!file) error2(0, PSTR("problem autorunning from SD card"));
  VPtr<object, SPIRAMVAlloc> autorun.setRawNum*SDReadInt(file));
  file.close();
  if (autorun != nil) {
    loadimage(nil);
    apply(0, autorun, nil, nil);
  }
#else
  SPIFFS.begin();
  File file = SPIFFS.open("/ULISP.IMG", "r");
  if (!file) error2(0, PSTR("problem autorunning from SPIFFS"));
  VPtr<object, SPIRAMVAlloc> autorun = VPtr<object, SPIRAMVAlloc> { };
  autorun.setRawNum(SpiffsReadInt(file));
  file.close();
  if (autorun != nil) {
    loadimage(nil);
    apply(0, autorun, nil, nil);
  }
#endif
}

// Error handling

void errorsub (symbol_t fname, PGM_P string) {
  pfl(pserial); pfstring(PSTR("Error: "), pserial);
  if (fname) {
    pserial('\''); 
    pstring(symbolname(fname), pserial);
    pfstring(PSTR("' "), pserial);
  }
  pfstring(string, pserial);
}

void error (symbol_t fname, PGM_P string, VPtr<object, SPIRAMVAlloc> symbol) {
  errorsub(fname, string);
  pfstring(PSTR(": "), pserial); printobject(symbol, pserial);
  pln(pserial);
  GCStack = nil;
  longjmp(exception, 1);
}

void error2 (symbol_t fname, PGM_P string) {
  errorsub(fname, string);
  pln(pserial);
  GCStack = nil;
  longjmp(exception, 1);
}

// Save space as these are used multiple times
const char notanumber[] PROGMEM = "argument is not a number";
const char notastring[] PROGMEM = "argument is not a string";
const char notalist[] PROGMEM = "argument is not a list";
const char notproper[] PROGMEM = "argument is not a proper list";
const char noargument[] PROGMEM = "missing argument";
const char nostream[] PROGMEM = "missing stream argument";
const char overflow[] PROGMEM = "arithmetic overflow";
const char invalidpin[] PROGMEM = "invalid pin";
const char resultproper[] PROGMEM = "result is not a proper list";

// Tracing

boolean tracing (symbol_t name) {
  int i = 0;
  while (i < TRACEMAX) {
    if (TraceFn[i] == name) return i+1;
    i++;
  }
  return 0;
}

void trace (symbol_t name) {
  if (tracing(name)) error(TRACE, PSTR("already being traced"), symbol(name));
  int i = 0;
  while (i < TRACEMAX) {
    if (TraceFn[i] == 0) { TraceFn[i] = name; TraceDepth[i] = 0; return; }
    i++;
  }
  error2(TRACE, PSTR("already tracing 3 functions"));
}

void untrace (symbol_t name) {
  int i = 0;
  while (i < TRACEMAX) {
    if (TraceFn[i] == name) { TraceFn[i] = 0; return; }
    i++;
  }
  error(UNTRACE, PSTR("not tracing"), symbol(name));
}

// Helper functions

boolean consp (VPtr<object, SPIRAMVAlloc> x) {
  if (x == nil) return false;
  unsigned int type = x->val.type;
  return type >= PAIR || type == ZERO;
}

boolean atom (VPtr<object, SPIRAMVAlloc> x) {
  if (x == nil) return true;
  unsigned int type = x->val.type;
  return type < PAIR && type != ZERO;
}

boolean listp (VPtr<object, SPIRAMVAlloc> x) {
  if (x == nil) return true;
  unsigned int type = x->val.type;
  return type >= PAIR || type == ZERO;
}

boolean improperp (VPtr<object, SPIRAMVAlloc> x) {
  if (x == nil) return false;
  unsigned int type = x->val.type;
  return type < PAIR && type != ZERO;
}

int toradix40 (char ch) {
  if (ch == 0) return 0;
  if (ch >= '0' && ch <= '9') return ch-'0'+30;
  ch = ch | 0x20;
  if (ch >= 'a' && ch <= 'z') return ch-'a'+1;
  return -1; // Invalid
}

int fromradix40 (int n) {
  if (n >= 1 && n <= 26) return 'a'+n-1;
  if (n >= 30 && n <= 39) return '0'+n-30;
  return 0;
}

int pack40 (char *buffer) {
  return (((toradix40(buffer[0]) * 40) + toradix40(buffer[1])) * 40 + toradix40(buffer[2]));
}

boolean valid40 (char *buffer) {
 return (toradix40(buffer[0]) >= 0 && toradix40(buffer[1]) >= 0 && toradix40(buffer[2]) >= 0);
}

int digitvalue (char d) {
  if (d>='0' && d<='9') return d-'0';
  d = d | 0x20;
  if (d>='a' && d<='f') return d-'a'+10;
  return 16;
}

char *symbolname (symbol_t x) {
  if (x < ENDFUNCTIONS) return lookupbuiltin(x);
  else if (x >= 64000) return lookupsymbol(x);
  char *buffer = SymbolTop;
  buffer[3] = '\0';
  for (int n=2; n>=0; n--) {
    buffer[n] = fromradix40(x % 40);
    x = x / 40;
  }
  return buffer;
}

int checkinteger (symbol_t name, VPtr<object, SPIRAMVAlloc> obj) {
  if (!integerp(obj)) error(name, PSTR("argument is not an integer"), obj);
  return obj->val.integer;
}

float checkintfloat (symbol_t name, VPtr<object, SPIRAMVAlloc> obj){
  if (integerp(obj)) return obj->val.integer;
  if (floatp(obj)) return obj->val.single_float;
  error(name, notanumber, obj);
  return 0;
}

int checkchar (symbol_t name, VPtr<object, SPIRAMVAlloc> obj) {
  if (!characterp(obj)) error(name, PSTR("argument is not a character"), obj);
  return obj->val.integer;
}

int isstream (VPtr<object, SPIRAMVAlloc> obj){
  if (!streamp(obj)) error(0, PSTR("not a stream"), obj);
  return obj->val.integer;
}

int issymbol (VPtr<object, SPIRAMVAlloc> obj, symbol_t n) {
  return symbolp(obj) && obj->val.name == n;
}

void checkargs (symbol_t name, VPtr<object, SPIRAMVAlloc> args) {
  int nargs = listlength(name, args);
  if (name >= ENDFUNCTIONS) error(0, PSTR("not valid here"), symbol(name));
  if (nargs<lookupmin(name)) error2(name, PSTR("has too few arguments"));
  if (nargs>lookupmax(name)) error2(name, PSTR("has too many arguments"));
}

int eq (VPtr<object, SPIRAMVAlloc> arg1, VPtr<object, SPIRAMVAlloc> arg2) {
  if (arg1 == arg2) return true;  // Same object
  if ((arg1 == nil) || (arg2 == nil)) return false;  // Not both values
  if (arg1->ptr.cdr != arg2->ptr.cdr) return false;  // Different values
  if (symbolp(arg1) && symbolp(arg2)) return true;  // Same symbol
  if (integerp(arg1) && integerp(arg2)) return true;  // Same integer
  if (floatp(arg1) && floatp(arg2)) return true; // Same float
  if (characterp(arg1) && characterp(arg2)) return true;  // Same character
  return false;
}

int listlength (symbol_t name, VPtr<object, SPIRAMVAlloc> list) {
  int length = 0;
  while (list != nil) {
    if (improperp(list)) error2(name, notproper);
    list = cdr(list);
    length++;
  }
  return length;
}

// Association lists

VPtr<object, SPIRAMVAlloc> assoc (VPtr<object, SPIRAMVAlloc> key, VPtr<object, SPIRAMVAlloc> list) {
  while (list != nil) {
    if (improperp(list)) error(ASSOC, notproper, list);
    VPtr<object, SPIRAMVAlloc> pair = first(list);
    if (!listp(pair)) error(ASSOC, PSTR("element is not a list"), pair);
    if (pair != nil && eq(key,car(pair))) return pair;
    list = cdr(list);
  }
  return nil;
}

VPtr<object, SPIRAMVAlloc> delassoc (VPtr<object, SPIRAMVAlloc> key, VPtr<object, SPIRAMVAlloc> *alist) {
  VPtr<object, SPIRAMVAlloc> list = *alist;
  VPtr<object, SPIRAMVAlloc> prev = nil;
  while (list != nil) {
    VPtr<object, SPIRAMVAlloc> pair = first(list);
    if (eq(key,car(pair))) {
      if (prev == nil) *alist = cdr(list);
      else cdr(prev) = cdr(list);
      return key;
    }
    prev = list;
    list = cdr(list);
  }
  return nil;
}

// String utilities

void indent (int spaces, pfun_t pfun) {
  for (int i=0; i<spaces; i++) pfun(' ');
}

void buildstring (char ch, int *chars, VPtr<object, SPIRAMVAlloc> *head) {
  static VPtr<object, SPIRAMVAlloc> tail;
  static uint8_t shift;
  if (*chars == 0) {
    shift = (sizeof(int)-1)*8;
    *chars = ch<<shift;
    VPtr<object, SPIRAMVAlloc> cell = myalloc();
    if (*head == nil) *head = cell; else tail->ptr.car = cell;
    cell->ptr.car = nil;
    cell->val.integer = *chars;
    tail = cell;
  } else {
    shift = shift - 8;
    *chars = *chars | ch<<shift;
    tail->val.integer = *chars;
    if (shift == 0) *chars = 0;
  }
}

VPtr<object, SPIRAMVAlloc> readstring (char delim, gfun_t gfun) {
  VPtr<object, SPIRAMVAlloc> obj = myalloc();
  obj->val.type = STRING;
  int ch = gfun();
  if (ch == -1) return nil;
  VPtr<object, SPIRAMVAlloc> head = nil;
  int chars = 0;
  while ((ch != delim) && (ch != -1)) {
    if (ch == '\\') ch = gfun();
    buildstring(ch, &chars, &head);
    ch = gfun();
  }
  obj->ptr.cdr = head;
  return obj;
}

int stringlength (VPtr<object, SPIRAMVAlloc> form) {
  int length = 0;
  form = cdr(form);
  while (form != nil) {
    int chars = form->val.integer;
    for (int i=(sizeof(int)-1)*8; i>=0; i=i-8) {
      if (chars>>i & 0xFF) length++;
    }
    form = car(form);
  }
  return length;
}

char nthchar (VPtr<object, SPIRAMVAlloc> string, int n) {
  VPtr<object, SPIRAMVAlloc> arg = cdr(string);
  int top;
  if (sizeof(int) == 4) { top = n>>2; n = 3 - (n&3); }
  else { top = n>>1; n = 1 - (n&1); }
  for (int i=0; i<top; i++) {
    if (arg == nil) return 0;
    arg = car(arg);
  }
  if (arg == nil) return 0;
  return (arg->val.integer)>>(n*8) & 0xFF;
}

char *cstringbuf (VPtr<object, SPIRAMVAlloc> arg) {
  cstring(arg, SymbolTop, SYMBOLTABLESIZE-(SymbolTop-SymbolTable));
  return SymbolTop;
}

char *cstring (VPtr<object, SPIRAMVAlloc> form, char *buffer, int buflen) {
  int index = 0;
  form = cdr(form);
  while (form != nil) {
    int chars = form->val.integer;
    for (int i=(sizeof(int)-1)*8; i>=0; i=i-8) {
      char ch = chars>>i & 0xFF;
      if (ch) {
        if (index >= buflen-1) error2(0, PSTR("no room for string"));
        buffer[index++] = ch;
      }
    }
    form = car(form);
  }
  buffer[index] = '\0';
  return buffer;
}

VPtr<object, SPIRAMVAlloc> lispstring (char *s) {
  VPtr<object, SPIRAMVAlloc> obj = myalloc();
  obj->val.type = STRING;
  char ch = *s++;
  VPtr<object, SPIRAMVAlloc> head = nil;
  int chars = 0;
  while (ch) {
    if (ch == '\\') ch = *s++;
    buildstring(ch, &chars, &head);
    ch = *s++;
  }
  obj->ptr.cdr = head;
  return obj;
}

// Lookup variable in environment

VPtr<object, SPIRAMVAlloc> value (symbol_t n, VPtr<object, SPIRAMVAlloc> env) {
  while (env != nil) {
    VPtr<object, SPIRAMVAlloc> pair = car(env);
    if (pair != nil && car(pair)->val.name == n) return pair;
    env = cdr(env);
  }
  return nil;
}

VPtr<object, SPIRAMVAlloc> findvalue (VPtr<object, SPIRAMVAlloc> var, VPtr<object, SPIRAMVAlloc> env) {
  symbol_t varname = var->val.name;
  VPtr<object, SPIRAMVAlloc> pair = value(varname, env);
  if (pair == nil) pair = value(varname, GlobalEnv);
  if (pair == nil) error(0, PSTR("unknown variable"), var);
  return pair;
}

// Handling closures
  
VPtr<object, SPIRAMVAlloc> closure (int tc, symbol_t name, VPtr<object, SPIRAMVAlloc> state, VPtr<object, SPIRAMVAlloc> function, VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> *env) {
  int trace = 0;
  if (name) trace = tracing(name);
  if (trace) {
    indent(TraceDepth[trace-1]<<1, pserial);
    pint(TraceDepth[trace-1]++, pserial);
    pserial(':'); pserial(' '); pserial('('); pstring(symbolname(name), pserial);
  }
  VPtr<object, SPIRAMVAlloc> params = first(function);
  function = cdr(function);
  // Dropframe
  if (tc) {
    if (*env != nil && car(*env) == nil) {
      pop(*env);
      while (*env != nil && car(*env) != nil) pop(*env);
    } else push(nil, *env);
  }
  // Push state
  while (state != nil) {
    VPtr<object, SPIRAMVAlloc> pair = first(state);
    push(pair, *env);
    state = cdr(state);
  }
  // Add arguments to environment
  boolean optional = false;
  while (params != nil) {
    VPtr<object, SPIRAMVAlloc> value;
    VPtr<object, SPIRAMVAlloc> var = first(params);
    if (symbolp(var) && var->val.name == OPTIONAL) optional = true;  
    else {
      if (consp(var)) {
        if (!optional) error(name, PSTR("invalid default value"), var);
        if (args == nil) value = eval(second(var), *env);
        else { value = first(args); args = cdr(args); }
        var = first(var);
        if (!symbolp(var)) error(name, PSTR("illegal optional parameter"), var); 
      } else if (!symbolp(var)) {
        error2(name, PSTR("illegal parameter"));     
      } else if (var->val.name == AMPREST) {
        params = cdr(params);
        var = first(params);
        value = args;
        args = nil;
      } else {
        if (args == nil) {
          if (optional) value = nil; 
          else {
            if (name) error2(name, PSTR("has too few arguments"));
            else error2(0, PSTR("function has too few arguments"));
          }
        } else { value = first(args); args = cdr(args); }
      }
      push(cons(var,value), *env);
      if (trace) { pserial(' '); printobject(value, pserial); }
    }
    params = cdr(params);  
  }
  if (args != nil) {
    if (name) error2(name, PSTR("has too many arguments"));
    else error2(0, PSTR("function has too many arguments"));
  }
  if (trace) { pserial(')'); pln(pserial); }
  // Do an implicit progn
  if (tc) push(nil, *env);
  return tf_progn(function, *env);
}

VPtr<object, SPIRAMVAlloc> apply (symbol_t name, VPtr<object, SPIRAMVAlloc> function, VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  if (symbolp(function)) {
    symbol_t fname = function->val.name;
    checkargs(fname, args);
    return ((fn_ptr_type)lookupfn(fname))(args, env);
  }
  if (consp(function) && issymbol(car(function), LAMBDA)) {
    function = cdr(function);
    VPtr<object, SPIRAMVAlloc> result = closure(0, 0, nil, function, args, &env);
    return eval(result, env);
  }
  if (consp(function) && issymbol(car(function), CLOSURE)) {
    function = cdr(function);
    VPtr<object, SPIRAMVAlloc> result = closure(0, 0, car(function), cdr(function), args, &env);
    return eval(result, env);
  }
  error(name, PSTR("illegal function"), function);
  return nil;
}

// In-place operations

VPtr<object, SPIRAMVAlloc> *place (symbol_t name, VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  if (atom(args)) return &cdr(findvalue(args, env));
  VPtr<object, SPIRAMVAlloc> function = first(args);
  if (issymbol(function, CAR) || issymbol(function, FIRST)) {
    VPtr<object, SPIRAMVAlloc> value = eval(second(args), env);
    if (!listp(value)) error(name, PSTR("can't take car"), value);
    return &car(value);
  }
  if (issymbol(function, CDR) || issymbol(function, REST)) {
    VPtr<object, SPIRAMVAlloc> value = eval(second(args), env);
    if (!listp(value)) error(name, PSTR("can't take cdr"), value);
    return &cdr(value);
  }
  if (issymbol(function, NTH)) {
    int index = checkinteger(NTH, eval(second(args), env));
    VPtr<object, SPIRAMVAlloc> list = eval(third(args), env);
    if (atom(list)) error(name, PSTR("second argument to nth is not a list"), list);
    while (index > 0) {
      list = cdr(list);
      if (list == nil) error2(name, PSTR("index to nth is out of range"));
      index--;
    }
    return &car(list);
  }
  error2(name, PSTR("illegal place"));
  return &nil;
}

// Checked car and cdr

inline VPtr<object, SPIRAMVAlloc> carx (VPtr<object, SPIRAMVAlloc> arg) {
  if (!listp(arg)) error(0, PSTR("Can't take car"), arg);
  if (arg == nil) return nil;
  return car(arg);
}

inline VPtr<object, SPIRAMVAlloc> cdrx (VPtr<object, SPIRAMVAlloc> arg) {
  if (!listp(arg)) error(0, PSTR("Can't take cdr"), arg);
  if (arg == nil) return nil;
  return cdr(arg);
}

// I2C interface

void I2Cinit (bool enablePullup) {
  (void) enablePullup;
  Wire.begin();
}

inline uint8_t I2Cread () {
  return Wire.read();
}

inline bool I2Cwrite (uint8_t data) {
  return Wire.write(data);
}

bool I2Cstart (uint8_t address, uint8_t read) {
 int ok = true;
 if (read == 0) {
   Wire.beginTransmission(address);
   ok = (Wire.endTransmission(true) == 0);
   Wire.beginTransmission(address);
 }
 else Wire.requestFrom(address, I2CCount);
 return ok;
}

bool I2Crestart (uint8_t address, uint8_t read) {
  int error = (Wire.endTransmission(false) != 0);
  if (read == 0) Wire.beginTransmission(address);
  else Wire.requestFrom(address, I2CCount);
  return error ? false : true;
}

void I2Cstop (uint8_t read) {
  if (read == 0) Wire.endTransmission(); // Check for error?
}

// Streams

inline int spiread () { return SPI.transfer(0); }
inline int serial1read () { while (!Serial1.available()) testescape(); return Serial1.read(); }
#if defined(sdcardsupport)
File SDpfile, SDgfile;
inline int SDread () {
  if (LastChar) { 
    char temp = LastChar;
    LastChar = 0;
    return temp;
  }
  return SDgfile.read();
}
#endif

File SPIFFSpfile, SPIFFSgfile;
inline int SPIFFSread () {
  if (LastChar) { 
    char temp = LastChar;
    LastChar = 0;
    return temp;
  }
  return SPIFFSgfile.read();
}

WiFiClient client;
WiFiServer server(80);

inline int WiFiread () {
  if (LastChar) { 
    char temp = LastChar;
    LastChar = 0;
    return temp;
  }
  return client.read();
}

void serialbegin (int address, int baud) {
  if (address == 1) Serial1.begin((long)baud*100);
  else error(WITHSERIAL, PSTR("port not supported"), number(address));
}

void serialend (int address) {
  if (address == 1) {Serial1.flush(); Serial1.end(); }
}

gfun_t gstreamfun (VPtr<object, SPIRAMVAlloc> args) {
  int streamtype = SERIALSTREAM;
  int address = 0;
  gfun_t gfun = gserial;
  if (args != nil) {
    int stream = isstream(first(args));
    streamtype = stream>>8; address = stream & 0xFF;
  }
  if (streamtype == I2CSTREAM) gfun = (gfun_t)I2Cread;
  else if (streamtype == SPISTREAM) gfun = spiread;
  else if (streamtype == SERIALSTREAM) {
    if (address == 0) gfun = gserial;
    else if (address == 1) gfun = serial1read;
  }
  #if defined(sdcardsupport)
  else if (streamtype == SDSTREAM) gfun = (gfun_t)SDread;
  #endif
  else if (streamtype == SPIFFSSTREAM) gfun = (gfun_t)SPIFFSread;
  else if (streamtype == WIFISTREAM) gfun = (gfun_t)WiFiread;
  else error2(0, PSTR("unknown stream type"));
  return gfun;
}

inline void spiwrite (char c) { SPI.transfer(c); }
inline void serial1write (char c) { Serial1.write(c); }
inline void WiFiwrite (char c) { client.write(c); }
#if defined(sdcardsupport)
inline void SDwrite (char c) { SDpfile.write(c); }
#endif
inline void SPIFFSwrite (char c) { SPIFFSpfile.write(c); }

pfun_t pstreamfun (VPtr<object, SPIRAMVAlloc> args) {
  int streamtype = SERIALSTREAM;
  int address = 0;
  pfun_t pfun = pserial;
  if (args != nil && first(args) != nil) {
    int stream = isstream(first(args));
    streamtype = stream>>8; address = stream & 0xFF;
  }
  if (streamtype == I2CSTREAM) pfun = (pfun_t)I2Cwrite;
  else if (streamtype == SPISTREAM) pfun = spiwrite;
  else if (streamtype == SERIALSTREAM) {
    if (address == 0) pfun = pserial;
    else if (address == 1) pfun = serial1write;
  }   
  #if defined(sdcardsupport)
  else if (streamtype == SDSTREAM) pfun = (pfun_t)SDwrite;
  #endif
  else if (streamtype == SPIFFSSTREAM) pfun = (pfun_t)SPIFFSwrite;
  else if (streamtype == WIFISTREAM) pfun = (pfun_t)WiFiwrite;
  else error2(0, PSTR("unknown stream type"));
  return pfun;
}

// Check pins

void checkanalogread (int pin) {
#if defined(ESP8266)
  if (pin!=17) error(ANALOGREAD, PSTR("invalid pin"), number(pin));
#elif defined(ESP32)
  if (!(pin==0 || pin==2 || pin==4 || (pin>=12 && pin<=15) || (pin>=25 && pin<=27) || (pin>=32 && pin<=36) || pin==39))
    error(ANALOGREAD, PSTR("invalid pin"), number(pin));
#endif
}

void checkanalogwrite (int pin) {
#if defined(ESP8266)
  if (!(pin>=0 && pin<=16)) error(ANALOGWRITE, PSTR("invalid pin"), number(pin));
#elif defined(ESP32)
  if (!(pin>=25 && pin<=26)) error(ANALOGWRITE, PSTR("invalid pin"), number(pin));
#endif
}

// Note

void tone (int pin, int note) {
  (void) pin, (void) note;
}

void noTone (int pin) {
  (void) pin;
}

const int scale[] PROGMEM = {4186,4435,4699,4978,5274,5588,5920,6272,6645,7040,7459,7902};

void playnote (int pin, int note, int octave) {
  int prescaler = 8 - octave - note/12;
  if (prescaler<0 || prescaler>8) error(NOTE, PSTR("octave out of range"), number(prescaler));
  tone(pin, scale[note%12]>>prescaler);
}

void nonote (int pin) {
  noTone(pin);
}

// Sleep

void initsleep () { }

void sleep (int secs) {
  delay(1000 * secs);
}

// Special forms

VPtr<object, SPIRAMVAlloc> sp_quote (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  checkargs(QUOTE, args); 
  return first(args);
}

VPtr<object, SPIRAMVAlloc> sp_defun (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  checkargs(DEFUN, args);
  VPtr<object, SPIRAMVAlloc> var = first(args);
  if (var->val.type != SYMBOL) error(DEFUN, PSTR("not a symbol"), var);
  VPtr<object, SPIRAMVAlloc> val = cons(symbol(LAMBDA), cdr(args));
  VPtr<object, SPIRAMVAlloc> pair = value(var->val.name,GlobalEnv);
  if (pair != nil) { cdr(pair) = val; return var; }
  push(cons(var, val), GlobalEnv);
  return var;
}

VPtr<object, SPIRAMVAlloc> sp_defvar (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  checkargs(DEFVAR, args);
  VPtr<object, SPIRAMVAlloc> var = first(args);
  if (var->val.type != SYMBOL) error(DEFVAR, PSTR("not a symbol"), var);
  VPtr<object, SPIRAMVAlloc> val = nil;
  val = eval(second(args), env);
  VPtr<object, SPIRAMVAlloc> pair = value(var->val.name, GlobalEnv);
  if (pair != nil) { cdr(pair) = val; return var; }
  push(cons(var, val), GlobalEnv);
  return var;
}

VPtr<object, SPIRAMVAlloc> sp_setq (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  VPtr<object, SPIRAMVAlloc> arg = nil;
  while (args != nil) {
    if (cdr(args) == nil) error2(SETQ, PSTR("odd number of parameters"));
    VPtr<object, SPIRAMVAlloc> pair = findvalue(first(args), env);
    arg = eval(second(args), env);
    cdr(pair) = arg;
    args = cddr(args);
  }
  return arg;
}

VPtr<object, SPIRAMVAlloc> sp_loop (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  VPtr<object, SPIRAMVAlloc> start = args;
  for (;;) {
    yield();
    args = start;
    while (args != nil) {
      VPtr<object, SPIRAMVAlloc> result = eval(car(args),env);
      if (tstflag(RETURNFLAG)) {
        clrflag(RETURNFLAG);
        return result;
      }
      args = cdr(args);
    }
  }
}

VPtr<object, SPIRAMVAlloc> sp_return (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  VPtr<object, SPIRAMVAlloc> result = eval(tf_progn(args,env), env);
  setflag(RETURNFLAG);
  return result;
}

VPtr<object, SPIRAMVAlloc> sp_push (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  checkargs(PUSH, args); 
  VPtr<object, SPIRAMVAlloc> item = eval(first(args), env);
  VPtr<object, SPIRAMVAlloc> *loc = place(PUSH, second(args), env);
  push(item, *loc);
  return *loc;
}

VPtr<object, SPIRAMVAlloc> sp_pop (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  checkargs(POP, args); 
  VPtr<object, SPIRAMVAlloc> *loc = place(POP, first(args), env);
  VPtr<object, SPIRAMVAlloc> result = car(*loc);
  pop(*loc);
  return result;
}

// Special forms incf/decf

VPtr<object, SPIRAMVAlloc> sp_incf (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  checkargs(INCF, args); 
  VPtr<object, SPIRAMVAlloc> *loc = place(INCF, first(args), env);
  args = cdr(args);
  
  VPtr<object, SPIRAMVAlloc> x = *loc;
  VPtr<object, SPIRAMVAlloc> inc = (args != nil) ? eval(first(args), env) : nil;

  if (floatp(x) || floatp(inc)) {
    float increment;
    float value = checkintfloat(INCF, x);

    if (inc == nil) increment = 1.0;
    else increment = checkintfloat(INCF, inc);

    *loc = makefloat(value + increment);
  } else if (integerp(x) && (integerp(inc) || inc == nil)) {
    int increment;
    int value = x->val.integer;

    if (inc == nil) increment = 1;
    else increment = inc->val.integer;

    if (increment < 1) {
      if (INT_MIN - increment > value) *loc = makefloat((float)value + (float)increment);
      else *loc = number(value + increment);
    } else {
      if (INT_MAX - increment < value) *loc = makefloat((float)value + (float)increment);
      else *loc = number(value + increment);
    }
  } else error2(INCF, notanumber);
  return *loc;
}

VPtr<object, SPIRAMVAlloc> sp_decf (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  checkargs(DECF, args); 
  VPtr<object, SPIRAMVAlloc> *loc = place(DECF, first(args), env);
  args = cdr(args);
  
  VPtr<object, SPIRAMVAlloc> x = *loc;
  VPtr<object, SPIRAMVAlloc> dec = (args != nil) ? eval(first(args), env) : nil;

  if (floatp(x) || floatp(dec)) {
    float decrement;
    float value = checkintfloat(DECF, x);

    if (dec == nil) decrement = 1.0;
    else decrement = checkintfloat(DECF, dec);

    *loc = makefloat(value - decrement);
  } if (integerp(x) && (integerp(dec) || dec == nil)) {
    int decrement;
    int value = x->val.integer;

    if (dec == nil) decrement = 1;
    else decrement = dec->val.integer;

    if (decrement < 1) {
      if (INT_MAX + decrement < value) *loc = makefloat((float)value - (float)decrement);
      else *loc = number(value - decrement);
    } else {
      if (INT_MIN + decrement > value) *loc = makefloat((float)value - (float)decrement);
      else *loc = number(value - decrement);
    }
  } else error2(DECF, notanumber);
  return *loc;
}

VPtr<object, SPIRAMVAlloc> sp_setf (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  VPtr<object, SPIRAMVAlloc> arg = nil;
  while (args != nil) {
    if (cdr(args) == nil) error2(SETF, PSTR("odd number of parameters"));
    VPtr<object, SPIRAMVAlloc> *loc = place(SETF, first(args), env);
    arg = eval(second(args), env);
    *loc = arg;
    args = cddr(args);
  }
  return arg;
}

VPtr<object, SPIRAMVAlloc> sp_dolist (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  if (args == nil) error2(DOLIST, noargument);
  VPtr<object, SPIRAMVAlloc> params = first(args);
  VPtr<object, SPIRAMVAlloc> var = first(params);
  VPtr<object, SPIRAMVAlloc> list = eval(second(params), env);
  push(list, GCStack); // Don't GC the list
  VPtr<object, SPIRAMVAlloc> pair = cons(var,nil);
  push(pair,env);
  params = cdr(cdr(params));
  args = cdr(args);
  while (list != nil) {
    if (improperp(list)) error(DOLIST, notproper, list);
    cdr(pair) = first(list);
    VPtr<object, SPIRAMVAlloc> forms = args;
    while (forms != nil) {
      VPtr<object, SPIRAMVAlloc> result = eval(car(forms), env);
      if (tstflag(RETURNFLAG)) {
        clrflag(RETURNFLAG);
        pop(GCStack);
        return result;
      }
      forms = cdr(forms);
    }
    list = cdr(list);
  }
  cdr(pair) = nil;
  pop(GCStack);
  if (params == nil) return nil;
  return eval(car(params), env);
}

VPtr<object, SPIRAMVAlloc> sp_dotimes (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  if (args == nil) error2(DOTIMES, noargument);
  VPtr<object, SPIRAMVAlloc> params = first(args);
  VPtr<object, SPIRAMVAlloc> var = first(params);
  int count = checkinteger(DOTIMES, eval(second(params), env));
  int index = 0;
  params = cdr(cdr(params));
  VPtr<object, SPIRAMVAlloc> pair = cons(var,number(0));
  push(pair,env);
  args = cdr(args);
  while (index < count) {
    cdr(pair) = number(index);
    VPtr<object, SPIRAMVAlloc> forms = args;
    while (forms != nil) {
      VPtr<object, SPIRAMVAlloc> result = eval(car(forms), env);
      if (tstflag(RETURNFLAG)) {
        clrflag(RETURNFLAG);
        return result;
      }
      forms = cdr(forms);
    }
    index++;
  }
  cdr(pair) = number(index);
  if (params == nil) return nil;
  return eval(car(params), env);
}

VPtr<object, SPIRAMVAlloc> sp_trace (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  while (args != nil) {
      trace(first(args)->val.name);
      args = cdr(args);
  }
  int i = 0;
  while (i < TRACEMAX) {
    if (TraceFn[i] != 0) args = cons(symbol(TraceFn[i]), args);
    i++;
  }
  return args;
}

VPtr<object, SPIRAMVAlloc> sp_untrace (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  if (args == nil) {
    int i = 0;
    while (i < TRACEMAX) {
      if (TraceFn[i] != 0) args = cons(symbol(TraceFn[i]), args);
      TraceFn[i] = 0;
      i++;
    }
  } else {
    while (args != nil) {
      untrace(first(args)->val.name);
      args = cdr(args);
    }
  }
  return args;
}

VPtr<object, SPIRAMVAlloc> sp_formillis (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  VPtr<object, SPIRAMVAlloc> param = first(args);
  unsigned long start = millis();
  unsigned long now, total = 0;
  if (param != nil) total = checkinteger(FORMILLIS, eval(first(param), env));
  eval(tf_progn(cdr(args),env), env);
  do {
    now = millis() - start;
    testescape();
  } while (now < total);
  if (now <= INT_MAX) return number(now);
  return nil;
}

VPtr<object, SPIRAMVAlloc> sp_withserial (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  VPtr<object, SPIRAMVAlloc> params = first(args);
  if (params == nil) error2(WITHSERIAL, nostream);
  VPtr<object, SPIRAMVAlloc> var = first(params);
  int address = checkinteger(WITHSERIAL, eval(second(params), env));
  params = cddr(params);
  int baud = 96;
  if (params != nil) baud = checkinteger(WITHSERIAL, eval(first(params), env));
  VPtr<object, SPIRAMVAlloc> pair = cons(var, stream(SERIALSTREAM, address));
  push(pair,env);
  serialbegin(address, baud);
  VPtr<object, SPIRAMVAlloc> forms = cdr(args);
  VPtr<object, SPIRAMVAlloc> result = eval(tf_progn(forms,env), env);
  serialend(address);
  return result;
}

VPtr<object, SPIRAMVAlloc> sp_withi2c (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  VPtr<object, SPIRAMVAlloc> params = first(args);
  if (params == nil) error2(WITHI2C, nostream);
  VPtr<object, SPIRAMVAlloc> var = first(params);
  int address = checkinteger(WITHI2C, eval(second(params), env));
  params = cddr(params);
  int read = 0; // Write
  I2CCount = 0;
  if (params != nil) {
    VPtr<object, SPIRAMVAlloc> rw = eval(first(params), env);
    if (integerp(rw)) I2CCount = rw->val.integer;
    read = (rw != nil);
  }
  I2Cinit(1); // Pullups
  VPtr<object, SPIRAMVAlloc> pair = cons(var, (I2Cstart(address, read)) ? stream(I2CSTREAM, address) : nil);
  push(pair,env);
  VPtr<object, SPIRAMVAlloc> forms = cdr(args);
  VPtr<object, SPIRAMVAlloc> result = eval(tf_progn(forms,env), env);
  I2Cstop(read);
  return result;
}

VPtr<object, SPIRAMVAlloc> sp_withspi (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  VPtr<object, SPIRAMVAlloc> params = first(args);
  if (params == nil) error2(WITHSPI, nostream);
  VPtr<object, SPIRAMVAlloc> var = first(params);
  params = cdr(params);
  if (params == nil) error2(WITHSPI, nostream);
  int pin = checkinteger(WITHSPI, eval(car(params), env));
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  params = cdr(params);
  int clock = 4000, mode = SPI_MODE0; // Defaults
  BitOrder bitorder = MSBFIRST;
  if (params != nil) {
    clock = checkinteger(WITHSPI, eval(car(params), env));
    params = cdr(params);
    if (params != nil) {
      bitorder = (checkinteger(WITHSPI, eval(car(params), env)) == 0) ? LSBFIRST : MSBFIRST;
      params = cdr(params);
      if (params != nil) {
        int modeval = checkinteger(WITHSPI, eval(car(params), env));
        mode = (modeval == 3) ? SPI_MODE3 : (modeval == 2) ? SPI_MODE2 : (modeval == 1) ? SPI_MODE1 : SPI_MODE0;
      }
    }
  }
  VPtr<object, SPIRAMVAlloc> pair = cons(var, stream(SPISTREAM, pin));
  push(pair,env);
  SPI.begin();
  SPI.beginTransaction(SPISettings(((unsigned long)clock * 1000), bitorder, mode));
  digitalWrite(pin, LOW);
  VPtr<object, SPIRAMVAlloc> forms = cdr(args);
  VPtr<object, SPIRAMVAlloc> result = eval(tf_progn(forms,env), env);
  digitalWrite(pin, HIGH);
  SPI.endTransaction();
  return result;
}

VPtr<object, SPIRAMVAlloc> sp_withsdcard (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
#if defined(sdcardsupport)
  VPtr<object, SPIRAMVAlloc> params = first(args);
  VPtr<object, SPIRAMVAlloc> var = first(params);
  VPtr<object, SPIRAMVAlloc> filename = eval(second(params), env);
  params = cddr(params);
  SD.begin();
  int mode = 0;
  if (params != nil && first(params) != nil) mode = checkinteger(WITHSDCARD, first(params));
  const char *oflag = FILE_READ;
  if (mode == 1) oflag = FILE_APPEND; else if (mode == 2) oflag = FILE_WRITE;
  if (mode >= 1) {
    SDpfile = SD.open(MakeFilename(filename), oflag);
    if (!SDpfile) error2(WITHSDCARD, PSTR("problem writing to SD card"));
  } else {
    SDgfile = SD.open(MakeFilename(filename), oflag);
    if (!SDgfile) error2(WITHSDCARD, PSTR("problem reading from SD card"));
  }
  VPtr<object, SPIRAMVAlloc> pair = cons(var, stream(SDSTREAM, 1));
  push(pair,env);
  VPtr<object, SPIRAMVAlloc> forms = cdr(args);
  VPtr<object, SPIRAMVAlloc> result = eval(tf_progn(forms,env), env);
  if (mode >= 1) SDpfile.close(); else SDgfile.close();
  return result;
#else
  (void) args, (void) env;
  error2(WITHSDCARD, PSTR("not supported"));
  return nil;
#endif
}

VPtr<object, SPIRAMVAlloc> sp_withspiffs (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  VPtr<object, SPIRAMVAlloc> params = first(args);
  VPtr<object, SPIRAMVAlloc> var = first(params);
  VPtr<object, SPIRAMVAlloc> filename = eval(second(params), env);
  params = cddr(params);
  SPIFFS.begin();
  int mode = 0;
  if (params != nil && first(params) != nil) mode = checkinteger(WITHSPIFFS, first(params));
  const char *oflag = "r";
  if (mode == 1) oflag = "a"; else if (mode == 2) oflag = "w";
  if (mode >= 1) {
    SPIFFSpfile = SPIFFS.open(MakeFilename(filename), oflag);
    if (!SPIFFSpfile) error2(WITHSPIFFS, PSTR("problem writing to SPIFFS"));
  } else {
    SPIFFSgfile = SPIFFS.open(MakeFilename(filename), oflag);
    if (!SPIFFSgfile) error2(WITHSPIFFS, PSTR("problem reading from SPIFFS"));
  }
  VPtr<object, SPIRAMVAlloc> pair = cons(var, stream(SPIFFSSTREAM, 1));
  push(pair,env);
  VPtr<object, SPIRAMVAlloc> forms = cdr(args);
  VPtr<object, SPIRAMVAlloc> result = eval(tf_progn(forms,env), env);
  if (mode >= 1) SPIFFSpfile.close(); else SPIFFSgfile.close();
  return result;
}

VPtr<object, SPIRAMVAlloc> sp_withclient (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  VPtr<object, SPIRAMVAlloc> params = first(args);
  VPtr<object, SPIRAMVAlloc> var = first(params);
  params = cdr(params);
  int n;
  if (params == nil) {
    client = server.available();
    if (!client) return nil;
    n = 2;
  } else {
    VPtr<object, SPIRAMVAlloc> address = eval(first(params), env);
    VPtr<object, SPIRAMVAlloc> port = eval(second(params), env);
    int success;
    if (stringp(address)) success = client.connect(cstringbuf(address), checkinteger(WITHCLIENT, port));
    else if (integerp(address)) success = client.connect(address->val.integer, checkinteger(WITHCLIENT, port));
    else error2(WITHCLIENT, PSTR("invalid address"));
    if (!success) return nil;
    n = 1;
  }
  VPtr<object, SPIRAMVAlloc> pair = cons(var, stream(WIFISTREAM, n));
  push(pair,env);
  VPtr<object, SPIRAMVAlloc> forms = cdr(args);
  VPtr<object, SPIRAMVAlloc> result = eval(tf_progn(forms,env), env);
  client.stop();
  return result;
}

// Tail-recursive forms

VPtr<object, SPIRAMVAlloc> tf_progn (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  if (args == nil) return nil;
  VPtr<object, SPIRAMVAlloc> more = cdr(args);
  while (more != nil) {
    VPtr<object, SPIRAMVAlloc> result = eval(car(args),env);
    if (tstflag(RETURNFLAG)) return result;
    args = more;
    more = cdr(args);
  }
  return car(args);
}

VPtr<object, SPIRAMVAlloc> tf_if (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  if (args == nil || cdr(args) == nil) error2(IF, PSTR("missing argument(s)"));
  if (eval(first(args), env) != nil) return second(args);
  args = cddr(args);
  return (args != nil) ? first(args) : nil;
}

VPtr<object, SPIRAMVAlloc> tf_cond (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  while (args != nil) {
    VPtr<object, SPIRAMVAlloc> clause = first(args);
    if (!consp(clause)) error(COND, PSTR("illegal clause"), clause);
    VPtr<object, SPIRAMVAlloc> test = eval(first(clause), env);
    VPtr<object, SPIRAMVAlloc> forms = cdr(clause);
    if (test != nil) {
      if (forms == nil) return test; else return tf_progn(forms, env);
    }
    args = cdr(args);
  }
  return nil;
}

VPtr<object, SPIRAMVAlloc> tf_when (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  if (args == nil) error2(WHEN, noargument);
  if (eval(first(args), env) != nil) return tf_progn(cdr(args),env);
  else return nil;
}

VPtr<object, SPIRAMVAlloc> tf_unless (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  if (args == nil) error2(UNLESS, noargument);
  if (eval(first(args), env) != nil) return nil;
  else return tf_progn(cdr(args),env);
}

VPtr<object, SPIRAMVAlloc> tf_case (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  VPtr<object, SPIRAMVAlloc> test = eval(first(args), env);
  args = cdr(args);
  while (args != nil) {
    VPtr<object, SPIRAMVAlloc> clause = first(args);
    if (!consp(clause)) error(CASE, PSTR("illegal clause"), clause);
    VPtr<object, SPIRAMVAlloc> key = car(clause);
    VPtr<object, SPIRAMVAlloc> forms = cdr(clause);
    if (consp(key)) {
      while (key != nil) {
        if (eq(test,car(key))) return tf_progn(forms, env);
        key = cdr(key);
      }
    } else if (eq(test,key) || eq(key,tee)) return tf_progn(forms, env);
    args = cdr(args);
  }
  return nil;
}

VPtr<object, SPIRAMVAlloc> tf_and (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  if (args == nil) return tee;
  VPtr<object, SPIRAMVAlloc> more = cdr(args);
  while (more != nil) {
    if (eval(car(args), env) == nil) return nil;
    args = more;
    more = cdr(args);
  }
  return car(args);
}

VPtr<object, SPIRAMVAlloc> tf_or (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  while (args != nil) {
    if (eval(car(args), env) != nil) return car(args);
    args = cdr(args);
  }
  return nil;
}

// Core functions

VPtr<object, SPIRAMVAlloc> fn_not (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return (first(args) == nil) ? tee : nil;
}

VPtr<object, SPIRAMVAlloc> fn_cons (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return cons(first(args), second(args));
}

VPtr<object, SPIRAMVAlloc> fn_atom (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return atom(first(args)) ? tee : nil;
}

VPtr<object, SPIRAMVAlloc> fn_listp (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return listp(first(args)) ? tee : nil;
}

VPtr<object, SPIRAMVAlloc> fn_consp (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return consp(first(args)) ? tee : nil;
}

VPtr<object, SPIRAMVAlloc> fn_symbolp (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  return symbolp(arg) ? tee : nil;
}

VPtr<object, SPIRAMVAlloc> fn_streamp (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  return streamp(arg) ? tee : nil;
}

VPtr<object, SPIRAMVAlloc> fn_eq (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return eq(first(args), second(args)) ? tee : nil;
}

// List functions

VPtr<object, SPIRAMVAlloc> fn_car (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return carx(first(args));
}

VPtr<object, SPIRAMVAlloc> fn_cdr (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return cdrx(first(args));
}

VPtr<object, SPIRAMVAlloc> fn_caar (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return carx(carx(first(args)));
}

VPtr<object, SPIRAMVAlloc> fn_cadr (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return carx(cdrx(first(args)));
}

VPtr<object, SPIRAMVAlloc> fn_cdar (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return cdrx(carx(first(args)));
}

VPtr<object, SPIRAMVAlloc> fn_cddr (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return cdrx(cdrx(first(args)));
}

VPtr<object, SPIRAMVAlloc> fn_caaar (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return carx(carx(carx(first(args))));
}

VPtr<object, SPIRAMVAlloc> fn_caadr (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return carx(carx(cdrx(first(args))));
}

VPtr<object, SPIRAMVAlloc> fn_cadar (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return carx(cdrx(carx(first(args))));
}

VPtr<object, SPIRAMVAlloc> fn_caddr (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return carx(cdrx(cdrx(first(args))));
}

VPtr<object, SPIRAMVAlloc> fn_cdaar (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return cdrx(carx(carx(first(args))));
}

VPtr<object, SPIRAMVAlloc> fn_cdadr (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return cdrx(carx(cdrx(first(args))));
}

VPtr<object, SPIRAMVAlloc> fn_cddar (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return cdrx(cdrx(carx(first(args))));
}

VPtr<object, SPIRAMVAlloc> fn_cdddr (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return cdrx(cdrx(cdrx(first(args))));
}

VPtr<object, SPIRAMVAlloc> fn_length (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  if (listp(arg)) return number(listlength(LENGTH, arg));
  if (!stringp(arg)) error(LENGTH, PSTR("argument is not a list or string"), arg);
  return number(stringlength(arg));
}

VPtr<object, SPIRAMVAlloc> fn_list (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return args;
}

VPtr<object, SPIRAMVAlloc> fn_reverse (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> list = first(args);
  VPtr<object, SPIRAMVAlloc> result = nil;
  while (list != nil) {
    if (improperp(list)) error(REVERSE, notproper, list);
    push(first(list),result);
    list = cdr(list);
  }
  return result;
}

VPtr<object, SPIRAMVAlloc> fn_nth (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  int n = checkinteger(NTH, first(args));
  VPtr<object, SPIRAMVAlloc> list = second(args);
  while (list != nil) {
    if (improperp(list)) error(NTH, notproper, list);
    if (n == 0) return car(list);
    list = cdr(list);
    n--;
  }
  return nil;
}

VPtr<object, SPIRAMVAlloc> fn_assoc (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> key = first(args);
  VPtr<object, SPIRAMVAlloc> list = second(args);
  return assoc(key,list);
}

VPtr<object, SPIRAMVAlloc> fn_member (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> item = first(args);
  VPtr<object, SPIRAMVAlloc> list = second(args);
  while (list != nil) {
    if (improperp(list)) error(MEMBER, notproper, list);
    if (eq(item,car(list))) return list;
    list = cdr(list);
  }
  return nil;
}

VPtr<object, SPIRAMVAlloc> fn_apply (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  VPtr<object, SPIRAMVAlloc> previous = nil;
  VPtr<object, SPIRAMVAlloc> last = args;
  while (cdr(last) != nil) {
    previous = last;
    last = cdr(last);
  }
  VPtr<object, SPIRAMVAlloc> arg = car(last);
  if (!listp(arg)) error(APPLY, PSTR("last argument is not a list"), arg);
  cdr(previous) = arg;
  return apply(APPLY, first(args), cdr(args), env);
}

VPtr<object, SPIRAMVAlloc> fn_funcall (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  return apply(FUNCALL, first(args), cdr(args), env);
}

VPtr<object, SPIRAMVAlloc> fn_append (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> head = nil;
  VPtr<object, SPIRAMVAlloc> tail;
  while (args != nil) {   
    VPtr<object, SPIRAMVAlloc> list = first(args);
    if (!listp(list)) error(APPEND, notalist, list);
    while (consp(list)) {
      VPtr<object, SPIRAMVAlloc> obj = cons(car(list), cdr(list));
      if (head == nil) head = obj;
      else cdr(tail) = obj;
      tail = obj;
      list = cdr(list);
      if (cdr(args) != nil && improperp(list)) error(APPEND, notproper, first(args));
    }
    args = cdr(args);
  }
  return head;
}

VPtr<object, SPIRAMVAlloc> fn_mapc (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  VPtr<object, SPIRAMVAlloc> function = first(args);
  args = cdr(args);
  VPtr<object, SPIRAMVAlloc> result = first(args);
  VPtr<object, SPIRAMVAlloc> params = cons(nil, nil);
  push(params,GCStack);
  // Make parameters
  while (true) {
    VPtr<object, SPIRAMVAlloc> tailp = params;
    VPtr<object, SPIRAMVAlloc> lists = args;
    while (lists != nil) {
      VPtr<object, SPIRAMVAlloc> list = car(lists);
      if (list == nil) {
         pop(GCStack);
         return result;
      }
      if (improperp(list)) error(MAPC, notproper, list);
      VPtr<object, SPIRAMVAlloc> obj = cons(first(list),nil);
      car(lists) = cdr(list);
      cdr(tailp) = obj; tailp = obj;
      lists = cdr(lists);
    }
    apply(MAPC, function, cdr(params), env);
  }
}

VPtr<object, SPIRAMVAlloc> fn_mapcar (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  VPtr<object, SPIRAMVAlloc> function = first(args);
  args = cdr(args);
  VPtr<object, SPIRAMVAlloc> params = cons(nil, nil);
  push(params,GCStack);
  VPtr<object, SPIRAMVAlloc> head = cons(nil, nil); 
  push(head,GCStack);
  VPtr<object, SPIRAMVAlloc> tail = head;
  // Make parameters
  while (true) {
    VPtr<object, SPIRAMVAlloc> tailp = params;
    VPtr<object, SPIRAMVAlloc> lists = args;
    while (lists != nil) {
      VPtr<object, SPIRAMVAlloc> list = car(lists);
      if (list == nil) {
         pop(GCStack);
         pop(GCStack);
         return cdr(head);
      }
      if (improperp(list)) error(MAPCAR, notproper, list);
      VPtr<object, SPIRAMVAlloc> obj = cons(first(list),nil);
      car(lists) = cdr(list);
      cdr(tailp) = obj; tailp = obj;
      lists = cdr(lists);
    }
    VPtr<object, SPIRAMVAlloc> result = apply(MAPCAR, function, cdr(params), env);
    VPtr<object, SPIRAMVAlloc> obj = cons(result,nil);
    cdr(tail) = obj; tail = obj;
  }
}

VPtr<object, SPIRAMVAlloc> fn_mapcan (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  VPtr<object, SPIRAMVAlloc> function = first(args);
  args = cdr(args);
  VPtr<object, SPIRAMVAlloc> params = cons(nil, nil);
  push(params,GCStack);
  VPtr<object, SPIRAMVAlloc> head = cons(nil, nil); 
  push(head,GCStack);
  VPtr<object, SPIRAMVAlloc> tail = head;
  // Make parameters
  while (true) {
    VPtr<object, SPIRAMVAlloc> tailp = params;
    VPtr<object, SPIRAMVAlloc> lists = args;
    while (lists != nil) {
      VPtr<object, SPIRAMVAlloc> list = car(lists);
      if (list == nil) {
         pop(GCStack);
         pop(GCStack);
         return cdr(head);
      }
      if (improperp(list)) error(MAPCAN, notproper, list);
      VPtr<object, SPIRAMVAlloc> obj = cons(first(list),nil);
      car(lists) = cdr(list);
      cdr(tailp) = obj; tailp = obj;
      lists = cdr(lists);
    }
    VPtr<object, SPIRAMVAlloc> result = apply(MAPCAN, function, cdr(params), env);
    while (consp(result)) {
      cdr(tail) = result; tail = result;
      result = cdr(result);
    }
    if (result != nil) error(MAPCAN, resultproper, result);
  }
}

// Arithmetic functions

VPtr<object, SPIRAMVAlloc> add_floats (VPtr<object, SPIRAMVAlloc> args, float fresult) {
  while (args != nil) {
    VPtr<object, SPIRAMVAlloc> arg = car(args);
    fresult = fresult + checkintfloat(ADD, arg);
    args = cdr(args);
  }
  return makefloat(fresult);
}

VPtr<object, SPIRAMVAlloc> fn_add (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  int result = 0;
  while (args != nil) {
    VPtr<object, SPIRAMVAlloc> arg = car(args);
    if (floatp(arg)) return add_floats(args, (float)result);
    else if (integerp(arg)) {
      int val = arg->val.integer;
      if (val < 1) { if (INT_MIN - val > result) return add_floats(args, (float)result); }
      else { if (INT_MAX - val < result) return add_floats(args, (float)result); }
      result = result + val;
    } else error(ADD, notanumber, arg);
    args = cdr(args);
  }
  return number(result);
}

VPtr<object, SPIRAMVAlloc> subtract_floats (VPtr<object, SPIRAMVAlloc> args, float fresult) {
  while (args != nil) {
    VPtr<object, SPIRAMVAlloc> arg = car(args);
    fresult = fresult - checkintfloat(SUBTRACT, arg);
    args = cdr(args);
  }
  return makefloat(fresult);
}

VPtr<object, SPIRAMVAlloc> negate (VPtr<object, SPIRAMVAlloc> arg) {
  if (integerp(arg)) {
    int result = arg->val.integer;
    if (result == INT_MIN) return makefloat(-result);
    else return number(-result);
  } else if (floatp(arg)) return makefloat(-(arg->val.single_float));
  else error(SUBTRACT, notanumber, arg);
  return nil;
}

VPtr<object, SPIRAMVAlloc> fn_subtract (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = car(args);
  args = cdr(args);
  if (args == nil) return negate(arg);
  else if (floatp(arg)) return subtract_floats(args, arg->val.single_float);
  else if (integerp(arg)) {
    int result = arg->val.integer;
    while (args != nil) {
      arg = car(args);
      if (floatp(arg)) return subtract_floats(args, result);
      else if (integerp(arg)) {
        int val = (car(args))->val.integer;
        if (val < 1) { if (INT_MAX + val < result) return subtract_floats(args, result); }
        else { if (INT_MIN + val > result) return subtract_floats(args, result); }
        result = result - val;
      } else error(SUBTRACT, notanumber, arg);
      args = cdr(args);
    }
    return number(result);
  } else error(SUBTRACT, notanumber, arg);
  return nil;
}

VPtr<object, SPIRAMVAlloc> multiply_floats (VPtr<object, SPIRAMVAlloc> args, float fresult) {
  while (args != nil) {
   VPtr<object, SPIRAMVAlloc> arg = car(args);
    fresult = fresult * checkintfloat(MULTIPLY, arg);
    args = cdr(args);
  }
  return makefloat(fresult);
}

VPtr<object, SPIRAMVAlloc> fn_multiply (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  int result = 1;
  while (args != nil){
    VPtr<object, SPIRAMVAlloc> arg = car(args);
    if (floatp(arg)) return multiply_floats(args, result);
    else if (integerp(arg)) {
      int64_t val = result * (int64_t)(arg->val.integer);
      if ((val > INT_MAX) || (val < INT_MIN)) return multiply_floats(args, result);
      result = val;
    } else error(MULTIPLY, notanumber, arg);
    args = cdr(args);
  }
  return number(result);
}

VPtr<object, SPIRAMVAlloc> divide_floats (VPtr<object, SPIRAMVAlloc> args, float fresult) {
  while (args != nil) {
    VPtr<object, SPIRAMVAlloc> arg = car(args);
    float f = checkintfloat(DIVIDE, arg);
    if (f == 0.0) error2(DIVIDE, PSTR("division by zero"));
    fresult = fresult / f;
    args = cdr(args);
  }
  return makefloat(fresult);
}

VPtr<object, SPIRAMVAlloc> fn_divide (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  args = cdr(args);
  // One argument
  if (args == nil) {
    if (floatp(arg)) {
      float f = arg->val.single_float;
      if (f == 0.0) error2(DIVIDE, PSTR("division by zero"));
      return makefloat(1.0 / f);
    } else if (integerp(arg)) {
      int i = arg->val.integer;
      if (i == 0) error2(DIVIDE, PSTR("division by zero"));
      else if (i == 1) return number(1);
      else return makefloat(1.0 / i);
    } else error(DIVIDE, notanumber, arg);
  }    
  // Multiple arguments
  if (floatp(arg)) return divide_floats(args, arg->val.single_float);
  else if (integerp(arg)) {
    int result = arg->val.integer;
    while (args != nil) {
      arg = car(args);
      if (floatp(arg)) {
        return divide_floats(args, result);
      } else if (integerp(arg)) {       
        int i = arg->val.integer;
        if (i == 0) error2(DIVIDE, PSTR("division by zero"));
        if ((result % i) != 0) return divide_floats(args, result);
        if ((result == INT_MIN) && (i == -1)) return divide_floats(args, result);
        result = result / i;
        args = cdr(args);
      } else error(DIVIDE, notanumber, arg);
    }
    return number(result); 
  } else error(DIVIDE, notanumber, arg);
  return nil;
}

VPtr<object, SPIRAMVAlloc> fn_mod (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg1 = first(args);
  VPtr<object, SPIRAMVAlloc> arg2 = second(args);
  if (integerp(arg1) && integerp(arg2)) {
    int divisor = arg2->val.integer;
    if (divisor == 0) error2(MOD, PSTR("division by zero"));
    int dividend = arg1->val.integer;
    int remainder = dividend % divisor;
    if ((dividend<0) != (divisor<0)) remainder = remainder + divisor;
    return number(remainder);
  } else {
    float fdivisor = checkintfloat(MOD, arg2);
    if (fdivisor == 0.0) error2(MOD, PSTR("division by zero"));
    float fdividend = checkintfloat(MOD, arg1);
    float fremainder = fmod(fdividend , fdivisor);
    if ((fdividend<0) != (fdivisor<0)) fremainder = fremainder + fdivisor;
    return makefloat(fremainder);
  }
}

VPtr<object, SPIRAMVAlloc> fn_oneplus (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  if (floatp(arg)) return makefloat((arg->val.single_float) + 1.0);
  else if (integerp(arg)) {
    int result = arg->val.integer;
    if (result == INT_MAX) return makefloat((arg->val.integer) + 1.0);
    else return number(result + 1);
  } else error(ONEPLUS, notanumber, arg);
  return nil;
}

VPtr<object, SPIRAMVAlloc> fn_oneminus (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  if (floatp(arg)) return makefloat((arg->val.single_float) - 1.0);
  else if (integerp(arg)) {
    int result = arg->val.integer;
    if (result == INT_MIN) return makefloat((arg->val.integer) - 1.0);
    else return number(result - 1);
  } else error(ONEMINUS, notanumber, arg);
  return nil;
}

VPtr<object, SPIRAMVAlloc> fn_abs (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  if (floatp(arg)) return makefloat(abs(arg->val.single_float));
  else if (integerp(arg)) {
    int result = arg->val.integer;
    if (result == INT_MIN) return makefloat(abs((float)result));
    else return number(abs(result));
  } else error(ABS, notanumber, arg);
  return nil;
}

VPtr<object, SPIRAMVAlloc> fn_random (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  if (integerp(arg)) return number(random(arg->val.integer));
  else if (floatp(arg)) return makefloat((float)rand()/(float)(RAND_MAX/(arg->val.single_float)));
  else error(RANDOM, notanumber, arg);
  return nil;
}

VPtr<object, SPIRAMVAlloc> fn_maxfn (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> result = first(args);
  args = cdr(args);
  while (args != nil) {
    VPtr<object, SPIRAMVAlloc> arg = car(args);
    if (integerp(result) && integerp(arg)) {
      if ((arg->val.integer) > (result->val.integer)) result = arg;
    } else if ((checkintfloat(MAXFN, arg) > checkintfloat(MAXFN, result))) result = arg;
    args = cdr(args); 
  }
  return result;
}

VPtr<object, SPIRAMVAlloc> fn_minfn (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> result = first(args);
  args = cdr(args);
  while (args != nil) {
    VPtr<object, SPIRAMVAlloc> arg = car(args);
    if (integerp(result) && integerp(arg)) {
      if ((arg->val.integer) < (result->val.integer)) result = arg;
    } else if ((checkintfloat(MINFN, arg) < checkintfloat(MINFN, result))) result = arg;
    args = cdr(args); 
  }
  return result;
}

// Arithmetic comparisons

VPtr<object, SPIRAMVAlloc> fn_noteq (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  while (args != nil) {
    VPtr<object, SPIRAMVAlloc> nargs = args;
    VPtr<object, SPIRAMVAlloc> arg1 = first(nargs);
    nargs = cdr(nargs);
    while (nargs != nil) {
      VPtr<object, SPIRAMVAlloc> arg2 = first(nargs);
      if (integerp(arg1) && integerp(arg2)) {
        if ((arg1->val.integer) == (arg2->val.integer)) return (VPtr<object, SPIRAMVAlloc>)nil;
      } else if ((checkintfloat(NOTEQ, arg1) == checkintfloat(NOTEQ, arg2))) return nil;
      nargs = cdr(nargs);
    }
    args = cdr(args);
  }
  return tee;
}

VPtr<object, SPIRAMVAlloc> fn_numeq (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg1 = first(args);
  args = cdr(args);
  while (args != nil) {
    VPtr<object, SPIRAMVAlloc> arg2 = first(args);
    if (integerp(arg1) && integerp(arg2)) {
      if (!((arg1->val.integer) == (arg2->val.integer))) return nil;
    } else if (!(checkintfloat(NUMEQ, arg1) == checkintfloat(NUMEQ, arg2))) return nil;
    arg1 = arg2;
    args = cdr(args);
  }
  return tee;
}

VPtr<object, SPIRAMVAlloc> fn_less (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg1 = first(args);
  args = cdr(args);
  while (args != nil) {
    VPtr<object, SPIRAMVAlloc> arg2 = first(args);
    if (integerp(arg1) && integerp(arg2)) {
      if (!((arg1->val.integer) < (arg2->val.integer))) return nil;
    } else if (!(checkintfloat(LESS, arg1) < checkintfloat(LESS, arg2))) return nil;
    arg1 = arg2;
    args = cdr(args);
  }
  return tee;
}

VPtr<object, SPIRAMVAlloc> fn_lesseq (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg1 = first(args);
  args = cdr(args);
  while (args != nil) {
    VPtr<object, SPIRAMVAlloc> arg2 = first(args);
    if (integerp(arg1) && integerp(arg2)) {
      if (!((arg1->val.integer) <= (arg2->val.integer))) return nil;
    } else if (!(checkintfloat(LESSEQ, arg1) <= checkintfloat(LESSEQ, arg2))) return nil;
    arg1 = arg2;
    args = cdr(args);
  }
  return tee;
}

VPtr<object, SPIRAMVAlloc> fn_greater (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg1 = first(args);
  args = cdr(args);
  while (args != nil) {
    VPtr<object, SPIRAMVAlloc> arg2 = first(args);
    if (integerp(arg1) && integerp(arg2)) {
      if (!((arg1->val.integer) > (arg2->val.integer))) return nil;
    } else if (!(checkintfloat(GREATER, arg1) > checkintfloat(GREATER, arg2))) return nil;
    arg1 = arg2;
    args = cdr(args);
  }
  return tee;
}

VPtr<object, SPIRAMVAlloc> fn_greatereq (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg1 = first(args);
  args = cdr(args);
  while (args != nil) {
    VPtr<object, SPIRAMVAlloc> arg2 = first(args);
    if (integerp(arg1) && integerp(arg2)) {
      if (!((arg1->val.integer) >= (arg2->val.integer))) return nil;
    } else if (!(checkintfloat(GREATEREQ, arg1) >= checkintfloat(GREATEREQ, arg2))) return nil;
    arg1 = arg2;
    args = cdr(args);
  }
  return tee;
}

VPtr<object, SPIRAMVAlloc> fn_plusp (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  if (floatp(arg)) return ((arg->val.single_float) > 0.0) ? tee : nil;
  else if (integerp(arg)) return ((arg->val.integer) > 0) ? tee : nil;
  else error(PLUSP, notanumber, arg);
  return nil;
}

VPtr<object, SPIRAMVAlloc> fn_minusp (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  if (floatp(arg)) return ((arg->val.single_float) < 0.0) ? tee : (VPtr<object, SPIRAMVAlloc>)nil;
  else if (integerp(arg)) return ((arg->val.integer) < 0) ? tee : nil;
  else error(MINUSP, notanumber, arg);
  return nil;
}

VPtr<object, SPIRAMVAlloc> fn_zerop (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  if (floatp(arg)) return (((arg->val.single_float) == 0.0) ? tee : nil);
  else if (integerp(arg)) return ((arg->val.integer) == 0) ? tee : nil;
  else error(ZEROP, notanumber, arg);
  return nil;
}

VPtr<object, SPIRAMVAlloc> fn_oddp (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  int arg = checkinteger(ODDP, first(args));
  return ((arg & 1) == 1) ? tee : nil;
}

VPtr<object, SPIRAMVAlloc> fn_evenp (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  int arg = checkinteger(EVENP, first(args));
  return ((arg & 1) == 0) ? tee : nil;
}

// Number functions

VPtr<object, SPIRAMVAlloc> fn_integerp (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return integerp(first(args)) ? tee : nil;
}

VPtr<object, SPIRAMVAlloc> fn_numberp (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  return (integerp(arg) || floatp(arg)) ? tee : nil;
}

// Floating-point functions

VPtr<object, SPIRAMVAlloc> fn_floatfn (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  return (floatp(arg)) ? arg : makefloat((float)(arg->val.integer));
}

VPtr<object, SPIRAMVAlloc> fn_floatp (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return floatp(first(args)) ? tee : nil;
}

VPtr<object, SPIRAMVAlloc> fn_sin (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return makefloat(sin(checkintfloat(SIN, first(args))));
}

VPtr<object, SPIRAMVAlloc> fn_cos (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return makefloat(cos(checkintfloat(COS, first(args))));
}

VPtr<object, SPIRAMVAlloc> fn_tan (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return makefloat(tan(checkintfloat(TAN, first(args))));
}

VPtr<object, SPIRAMVAlloc> fn_asin (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return makefloat(asin(checkintfloat(ASIN, first(args))));
}

VPtr<object, SPIRAMVAlloc> fn_acos (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return makefloat(acos(checkintfloat(ACOS, first(args))));
}

VPtr<object, SPIRAMVAlloc> fn_atan (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  float div = 1.0;
  args = cdr(args);
  if (args != nil) div = checkintfloat(ATAN, first(args));
  return makefloat(atan2(checkintfloat(ATAN, arg), div));
}

VPtr<object, SPIRAMVAlloc> fn_sinh (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return makefloat(sinh(checkintfloat(SINH, first(args))));
}

VPtr<object, SPIRAMVAlloc> fn_cosh (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return makefloat(cosh(checkintfloat(COSH, first(args))));
}

VPtr<object, SPIRAMVAlloc> fn_tanh (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return makefloat(tanh(checkintfloat(TANH, first(args))));
}

VPtr<object, SPIRAMVAlloc> fn_exp (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return makefloat(exp(checkintfloat(EXP, first(args))));
}

VPtr<object, SPIRAMVAlloc> fn_sqrt (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return makefloat(sqrt(checkintfloat(SQRT, first(args))));
}

VPtr<object, SPIRAMVAlloc> fn_log (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  float fresult = log(checkintfloat(LOG, arg));
  args = cdr(args);
  if (args == nil) return makefloat(fresult);
  else return makefloat(fresult / log(checkintfloat(LOG, first(args))));
}

int intpower (int base, int exp) {
  int result = 1;
  while (exp) {
    if (exp & 1) result = result * base;
    exp = exp / 2;
    base = base * base;
  }
  return result;
}

VPtr<object, SPIRAMVAlloc> fn_expt (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg1 = first(args); VPtr<object, SPIRAMVAlloc> arg2 = second(args);
  float float1 = checkintfloat(EXPT, arg1);
  float value = log(abs(float1)) * checkintfloat(EXPT, arg2);
  if (integerp(arg1) && integerp(arg2) && ((arg2->val.integer) > 0) && (abs(value) < 21.4875)) 
    return number(intpower(arg1->val.integer, arg2->val.integer));
  if (float1 < 0) error2(EXPT, PSTR("invalid result"));
  return makefloat(exp(value));
}

VPtr<object, SPIRAMVAlloc> fn_ceiling (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  args = cdr(args);
  if (args != nil) return number(ceil(checkintfloat(CEILING, arg) / checkintfloat(CEILING, first(args))));
  else return number(ceil(checkintfloat(CEILING, arg)));
}

VPtr<object, SPIRAMVAlloc> fn_floor (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  args = cdr(args);
  if (args != nil) return number(floor(checkintfloat(FLOOR, arg) / checkintfloat(FLOOR, first(args))));
  else return number(floor(checkintfloat(FLOOR, arg)));
}

VPtr<object, SPIRAMVAlloc> fn_truncate (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  args = cdr(args);
  if (args != nil) return number((int)(checkintfloat(TRUNCATE, arg) / checkintfloat(TRUNCATE, first(args))));
  else return number((int)(checkintfloat(TRUNCATE, arg)));
}

int myround (float number) {
  return (number >= 0) ? (int)(number + 0.5) : (int)(number - 0.5);
}

VPtr<object, SPIRAMVAlloc> fn_round (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  args = cdr(args);
  if (args != nil) return number(myround(checkintfloat(ROUND, arg) / checkintfloat(ROUND, first(args))));
  else return number(myround(checkintfloat(ROUND, arg)));
}

// Characters

VPtr<object, SPIRAMVAlloc> fn_char (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  if (!stringp(arg)) error(CHAR, notastring, arg);
  char c = nthchar(arg, checkinteger(CHAR, second(args)));
  if (c == 0) error2(CHAR, PSTR("index out of range"));
  return character(c);
}

VPtr<object, SPIRAMVAlloc> fn_charcode (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return number(checkchar(CHARCODE, first(args)));
}

VPtr<object, SPIRAMVAlloc> fn_codechar (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return character(checkinteger(CODECHAR, first(args)));
}

VPtr<object, SPIRAMVAlloc> fn_characterp (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return characterp(first(args)) ? tee : nil;
}

// Strings

VPtr<object, SPIRAMVAlloc> fn_stringp (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return stringp(first(args)) ? tee : nil;
}

bool stringcompare (symbol_t name, VPtr<object, SPIRAMVAlloc> args, bool lt, bool gt, bool eq) {
  VPtr<object, SPIRAMVAlloc> arg1 = first(args); if (!stringp(arg1)) error(name, notastring, arg1);
  VPtr<object, SPIRAMVAlloc> arg2 = second(args); if (!stringp(arg2)) error(name, notastring, arg2); 
  arg1 = cdr(arg1);
  arg2 = cdr(arg2);
  while ((arg1 != nil) || (arg2 != nil)) {
    if (arg1 == nil) return lt;
    if (arg2 == nil) return gt;
    if (arg1->val.integer < arg2->val.integer) return lt;
    if (arg1->val.integer > arg2->val.integer) return gt;
    arg1 = car(arg1);
    arg2 = car(arg2);
  }
  return eq;
}

VPtr<object, SPIRAMVAlloc> fn_stringeq (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return stringcompare(STRINGEQ, args, false, false, true) ? tee : nil;
}

VPtr<object, SPIRAMVAlloc> fn_stringless (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return stringcompare(STRINGLESS, args, true, false, false) ? tee : nil;
}

VPtr<object, SPIRAMVAlloc> fn_stringgreater (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  return stringcompare(STRINGGREATER, args, false, true, false) ? tee : nil;
}

VPtr<object, SPIRAMVAlloc> fn_sort (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  if (first(args) == nil) return nil;
  VPtr<object, SPIRAMVAlloc> list = cons(nil,first(args));
  push(list,GCStack);
  VPtr<object, SPIRAMVAlloc> predicate = second(args);
  VPtr<object, SPIRAMVAlloc> compare = cons(nil, cons(nil, nil));
  push(compare,GCStack);
  VPtr<object, SPIRAMVAlloc> ptr = cdr(list);
  while (cdr(ptr) != nil) {
    VPtr<object, SPIRAMVAlloc> go = list;
    while (go != ptr) {
      car(compare) = car(cdr(ptr));
      car(cdr(compare)) = car(cdr(go));
      if (apply(SORT, predicate, compare, env)) break;
      go = cdr(go);
    }
    if (go != ptr) {
      VPtr<object, SPIRAMVAlloc> obj = cdr(ptr);
      cdr(ptr) = cdr(obj);
      cdr(obj) = cdr(go);
      cdr(go) = obj;
    } else ptr = cdr(ptr);
  }
  pop(GCStack); pop(GCStack);
  return cdr(list);
}

VPtr<object, SPIRAMVAlloc> fn_stringfn (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  int type = arg->val.type;
  if (type == STRING) return arg;
  VPtr<object, SPIRAMVAlloc> obj = myalloc();
  obj->val.type = STRING;
  if (type == CHARACTER) {
    VPtr<object, SPIRAMVAlloc> cell = myalloc();
    cell->ptr.car = nil;
    uint8_t shift = (sizeof(int)-1)*8;
    cell->val.integer = (arg->val.integer)<<shift;
    obj->ptr.cdr = cell;
  } else if (type == SYMBOL) {
    char *s = symbolname(arg->val.name);
    char ch = *s++;
    VPtr<object, SPIRAMVAlloc> head = nil;
    int chars = 0;
    while (ch) {
      if (ch == '\\') ch = *s++;
      buildstring(ch, &chars, &head);
      ch = *s++;
    }
    obj->ptr.cdr = head;
  } else error(STRINGFN, PSTR("can't convert to string"), arg);
  return obj;
}

VPtr<object, SPIRAMVAlloc> fn_concatenate (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  symbol_t name = arg->val.name;
  if (name != STRINGFN) error2(CONCATENATE, PSTR("only supports strings"));
  args = cdr(args);
  VPtr<object, SPIRAMVAlloc> result = myalloc();
  result->val.type = STRING;
  VPtr<object, SPIRAMVAlloc> head = nil;
  int chars = 0;
  while (args != nil) {
    VPtr<object, SPIRAMVAlloc> obj = first(args);
    if (obj->val.type != STRING) error(CONCATENATE, notastring, obj);
    obj = cdr(obj);
    while (obj != nil) {
      int quad = obj->val.integer;
      while (quad != 0) {
         char ch = quad>>((sizeof(int)-1)*8) & 0xFF;
         buildstring(ch, &chars, &head);
         quad = quad<<8;
      }
      obj = car(obj);
    }
    args = cdr(args);
  }
  result->ptr.cdr = head;
  return result;
}

VPtr<object, SPIRAMVAlloc> fn_subseq (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  if (!stringp(arg)) error(SUBSEQ, notastring, arg);
  int start = checkinteger(SUBSEQ, second(args));
  int end;
  args = cddr(args);
  if (args != nil) end = checkinteger(SUBSEQ, car(args)); else end = stringlength(arg);
  VPtr<object, SPIRAMVAlloc> result = myalloc();
  result->val.type = STRING;
  VPtr<object, SPIRAMVAlloc> head = nil;
  int chars = 0;
  for (int i=start; i<end; i++) {
    char ch = nthchar(arg, i);
    if (ch == 0) error2(SUBSEQ, PSTR("index out of range"));
    buildstring(ch, &chars, &head);
  }
  result->ptr.cdr = head;
  return result;
}

int gstr () {
  if (LastChar) { 
    char temp = LastChar;
    LastChar = 0;
    return temp;
  }
  char c = nthchar(GlobalString, GlobalStringIndex++);
  return (c != 0) ? c : '\n'; // -1?
}

VPtr<object, SPIRAMVAlloc> fn_readfromstring (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {   
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  if (!stringp(arg)) error(READFROMSTRING, notastring, arg);
  GlobalString = arg;
  GlobalStringIndex = 0;
  return read(gstr);
}

void pstr (char c) {
  buildstring(c, &GlobalStringIndex, &GlobalString);
}
 
VPtr<object, SPIRAMVAlloc> fn_princtostring (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {   
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  VPtr<object, SPIRAMVAlloc> obj = myalloc();
  obj->val.type = STRING;
  GlobalString = nil;
  GlobalStringIndex = 0;
  char temp = Flags;
  clrflag(PRINTREADABLY);
  printobject(arg, pstr);
  Flags = temp;
  obj->ptr.cdr = GlobalString;
  return obj;
}

VPtr<object, SPIRAMVAlloc> fn_prin1tostring (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {   
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  VPtr<object, SPIRAMVAlloc> obj = myalloc();
  obj->val.type = STRING;
  GlobalString = nil;
  GlobalStringIndex = 0;
  printobject(arg, pstr);
  obj->ptr.cdr = GlobalString;
  return obj;
}

// Bitwise operators

VPtr<object, SPIRAMVAlloc> fn_logand (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  int result = -1;
  while (args != nil) {
    result = result & checkinteger(LOGAND, first(args));
    args = cdr(args);
  }
  return number(result);
}

VPtr<object, SPIRAMVAlloc> fn_logior (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  int result = 0;
  while (args != nil) {
    result = result | checkinteger(LOGIOR, first(args));
    args = cdr(args);
  }
  return number(result);
}

VPtr<object, SPIRAMVAlloc> fn_logxor (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  int result = 0;
  while (args != nil) {
    result = result ^ checkinteger(LOGXOR, first(args));
    args = cdr(args);
  }
  return number(result);
}

VPtr<object, SPIRAMVAlloc> fn_lognot (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  int result = checkinteger(LOGNOT, car(args));
  return number(~result);
}

VPtr<object, SPIRAMVAlloc> fn_ash (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  int value = checkinteger(ASH, first(args));
  int count = checkinteger(ASH, second(args));
  if (count >= 0) return number(value << count);
  else return number(value >> abs(count));
}

VPtr<object, SPIRAMVAlloc> fn_logbitp (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  int index = checkinteger(LOGBITP, first(args));
  int value = checkinteger(LOGBITP, second(args));
  return (bitRead(value, index) == 1) ? tee : nil;
}

// System functions

VPtr<object, SPIRAMVAlloc> fn_eval (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  return eval(first(args), env);
}

VPtr<object, SPIRAMVAlloc> fn_globals (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) args;
  if (GlobalEnv == nil) return nil;
  return fn_mapcar(cons(symbol(CAR),cons(GlobalEnv,nil)), env);
}

VPtr<object, SPIRAMVAlloc> fn_locals (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) args;
  return env;
}

VPtr<object, SPIRAMVAlloc> fn_makunbound (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> key = first(args);
  delassoc(key, &GlobalEnv);
  return key;
}

VPtr<object, SPIRAMVAlloc> fn_break (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) args;
  pfstring(PSTR("\rBreak!\r"), pserial);
  BreakLevel++;
  repl(env);
  BreakLevel--;
  return nil;
}

VPtr<object, SPIRAMVAlloc> fn_read (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  gfun_t gfun = gstreamfun(args);
  return read(gfun);
}

VPtr<object, SPIRAMVAlloc> fn_prin1 (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> obj = first(args);
  pfun_t pfun = pstreamfun(cdr(args));
  printobject(obj, pfun);
  return obj;
}

VPtr<object, SPIRAMVAlloc> fn_print (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> obj = first(args);
  pfun_t pfun = pstreamfun(cdr(args));
  pln(pfun);
  printobject(obj, pfun);
  (pfun)(' ');
  return obj;
}

VPtr<object, SPIRAMVAlloc> fn_princ (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> obj = first(args);
  pfun_t pfun = pstreamfun(cdr(args));
  char temp = Flags;
  clrflag(PRINTREADABLY);
  printobject(obj, pfun);
  Flags = temp;
  return obj;
}

VPtr<object, SPIRAMVAlloc> fn_terpri (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  pfun_t pfun = pstreamfun(args);
  pln(pfun);
  return nil;
}

VPtr<object, SPIRAMVAlloc> fn_readbyte (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  gfun_t gfun = gstreamfun(args);
  int c = gfun();
  return (c == -1) ? nil : number(c);
}

VPtr<object, SPIRAMVAlloc> fn_readline (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  gfun_t gfun = gstreamfun(args);
  return readstring('\n', gfun);
}

VPtr<object, SPIRAMVAlloc> fn_writebyte (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  int value = checkinteger(WRITEBYTE, first(args));
  pfun_t pfun = pstreamfun(cdr(args));
  (pfun)(value);
  return nil;
}

VPtr<object, SPIRAMVAlloc> fn_writestring (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> obj = first(args);
  pfun_t pfun = pstreamfun(cdr(args));
  char temp = Flags;
  clrflag(PRINTREADABLY);
  printstring(obj, pfun);
  Flags = temp;
  return nil;
}

VPtr<object, SPIRAMVAlloc> fn_writeline (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> obj = first(args);
  pfun_t pfun = pstreamfun(cdr(args));
  char temp = Flags;
  clrflag(PRINTREADABLY);
  printstring(obj, pfun);
  pln(pfun);
  Flags = temp;
  return nil;
}

VPtr<object, SPIRAMVAlloc> fn_restarti2c (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  int stream = first(args)->val.integer;
  args = cdr(args);
  int read = 0; // Write
  I2CCount = 0;
  if (args != nil) {
    VPtr<object, SPIRAMVAlloc> rw = first(args);
    if (integerp(rw)) I2CCount = rw->val.integer;
    read = (rw != nil);
  }
  int address = stream & 0xFF;
  if (stream>>8 != I2CSTREAM) error2(RESTARTI2C, PSTR("not an i2c stream"));
  return I2Crestart(address, read) ? tee : nil;
}

VPtr<object, SPIRAMVAlloc> fn_gc (VPtr<object, SPIRAMVAlloc> obj, VPtr<object, SPIRAMVAlloc> env) {
  int initial = Freespace;
  unsigned long start = micros();
  gc(obj, env);
  unsigned long elapsed = micros() - start;
  pfstring(PSTR("Space: "), pserial);
  pint(Freespace - initial, pserial);
  pfstring(PSTR(" bytes, Time: "), pserial);
  pint(elapsed, pserial);
  pfstring(PSTR(" us\r"), pserial);
  return nil;
}

VPtr<object, SPIRAMVAlloc> fn_room (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) args, (void) env;
  return number(Freespace);
}

VPtr<object, SPIRAMVAlloc> fn_saveimage (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  if (args != nil) args = eval(first(args), env);
  return number(saveimage(args));
}

VPtr<object, SPIRAMVAlloc> fn_loadimage (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  if (args != nil) args = first(args);
  return number(loadimage(args));
}

VPtr<object, SPIRAMVAlloc> fn_cls (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) args, (void) env;
  pserial(12);
  return nil;
}

// Arduino procedures

VPtr<object, SPIRAMVAlloc> fn_pinmode (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  int pin = checkinteger(PINMODE, first(args));
  PinMode pm = INPUT;
  VPtr<object, SPIRAMVAlloc> mode = second(args);
  if (integerp(mode)) {
    int nmode = mode->val.integer;
    if (nmode == 1) pm = OUTPUT; else if (nmode == 2) pm = INPUT_PULLUP;
    #if defined(INPUT_PULLDOWN)
    else if (nmode == 4) pm = INPUT_PULLDOWN;
    #endif
  } else if (mode != nil) pm = OUTPUT;
  pinMode(pin, pm);
  return nil;
}

VPtr<object, SPIRAMVAlloc> fn_digitalread (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  int pin = checkinteger(DIGITALREAD, first(args));
  if (digitalRead(pin) != 0) return tee; else return nil;
}

VPtr<object, SPIRAMVAlloc> fn_digitalwrite (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  int pin = checkinteger(DIGITALWRITE, first(args));
  VPtr<object, SPIRAMVAlloc> mode = second(args);
  if (integerp(mode)) digitalWrite(pin, mode->val.integer ? HIGH : LOW);
  else digitalWrite(pin, (mode != nil) ? HIGH : LOW);
  return mode;
}

VPtr<object, SPIRAMVAlloc> fn_analogread (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  int pin = checkinteger(ANALOGREAD, first(args));
  checkanalogread(pin);
  return number(analogRead(pin));
}
 
VPtr<object, SPIRAMVAlloc> fn_analogwrite (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  int pin = checkinteger(ANALOGWRITE, first(args));
  checkanalogwrite(pin);
  VPtr<object, SPIRAMVAlloc> value = second(args);
  analogWrite(pin, checkinteger(ANALOGWRITE, value));
  return value;
}

VPtr<object, SPIRAMVAlloc> fn_delay (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg1 = first(args);
  delay(checkinteger(DELAY, arg1));
  return arg1;
}

VPtr<object, SPIRAMVAlloc> fn_millis (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) args, (void) env;
  return number(millis());
}

VPtr<object, SPIRAMVAlloc> fn_sleep (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> arg1 = first(args);
  sleep(checkinteger(SLEEP, arg1));
  return arg1;
}

VPtr<object, SPIRAMVAlloc> fn_note (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  static int pin = 255;
  if (args != nil) {
    pin = checkinteger(NOTE, first(args));
    int note = 0;
    if (cddr(args) != nil) note = checkinteger(NOTE, second(args));
    int octave = 0;
    if (cddr(args) != nil) octave = checkinteger(NOTE, third(args));
    playnote(pin, note, octave);
  } else nonote(pin);
  return nil;
}

// Tree Editor

VPtr<object, SPIRAMVAlloc> fn_edit (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  VPtr<object, SPIRAMVAlloc> fun = first(args);
  VPtr<object, SPIRAMVAlloc> pair = findvalue(fun, env);
  clrflag(EXITEDITOR);
  VPtr<object, SPIRAMVAlloc> arg = edit(eval(fun, env));
  cdr(pair) = arg;
  return arg;
}

VPtr<object, SPIRAMVAlloc> edit (VPtr<object, SPIRAMVAlloc> fun) {
  while (1) {
    if (tstflag(EXITEDITOR)) return fun;
    char c = gserial();
    if (c == 'q') setflag(EXITEDITOR);
    else if (c == 'b') return fun;
    else if (c == 'r') fun = read(gserial);
    else if (c == '\n') { pfl(pserial); superprint(fun, 0, pserial); pln(pserial); }
    else if (c == 'c') fun = cons(read(gserial), fun);
    else if (atom(fun)) pserial('!');
    else if (c == 'd') fun = cons(car(fun), edit(cdr(fun)));
    else if (c == 'a') fun = cons(edit(car(fun)), cdr(fun));
    else if (c == 'x') fun = cdr(fun);
    else pserial('?');
  }
}

// Pretty printer

const int PPINDENT = 2;
const int PPWIDTH = 80;

void pcount (char c) {
  LastPrint = c;
  if (c == '\n') GlobalStringIndex++;
  GlobalStringIndex++;
}
  
int atomwidth (VPtr<object, SPIRAMVAlloc> obj) {
  GlobalStringIndex = 0;
  printobject(obj, pcount);
  return GlobalStringIndex;
}

boolean quoted (VPtr<object, SPIRAMVAlloc> obj) {
  return (consp(obj) && car(obj) != nil && car(obj)->val.name == QUOTE && consp(cdr(obj)) && cddr(obj) == nil);
}

int subwidth (VPtr<object, SPIRAMVAlloc> obj, int w) {
  if (atom(obj)) return w - atomwidth(obj);
  if (quoted(obj)) return subwidthlist(car(cdr(obj)), w - 1);
  return subwidthlist(obj, w - 1);
}

int subwidthlist (VPtr<object, SPIRAMVAlloc> form, int w) {
  while (form != nil && w >= 0) {
    if (atom(form)) return w - (2 + atomwidth(form));
    w = subwidth(car(form), w - 1);
    form = cdr(form);
  }
  return w;
}

void superprint (VPtr<object, SPIRAMVAlloc> form, int lm, pfun_t pfun) {
  if (atom(form)) {
    if (symbolp(form) && form->val.name == NOTHING) pstring(symbolname(form->val.name), pfun);
    else printobject(form, pfun);
  }
  else if (quoted(form)) { pfun('\''); superprint(car(cdr(form)), lm + 1, pfun); }
  else if (subwidth(form, PPWIDTH - lm) >= 0) supersub(form, lm + PPINDENT, 0, pfun);
  else supersub(form, lm + PPINDENT, 1, pfun);
}

const int ppspecials = 17;
const char ppspecial[ppspecials] PROGMEM = 
  { DOTIMES, DOLIST, IF, SETQ, TEE, LET, LETSTAR, LAMBDA, WHEN, UNLESS, WITHI2C, WITHSERIAL, WITHSPI, WITHSDCARD, WITHSPIFFS, FORMILLIS, WITHCLIENT };

void supersub (VPtr<object, SPIRAMVAlloc> form, int lm, int super, pfun_t pfun) {
  int special = 0, separate = 1;
  VPtr<object, SPIRAMVAlloc> arg = car(form);
  if (symbolp(arg)) {
    int name = arg->val.name;
    if (name == DEFUN) special = 2;
    else for (int i=0; i<ppspecials; i++) {
      if (name == ppspecial[i]) { special = 1; break; }   
    } 
  }
  while (form != nil) {
    if (atom(form)) { pfstring(PSTR(" . "), pfun); printobject(form, pfun); pfun(')'); return; }
    else if (separate) { pfun('('); separate = 0; }
    else if (special) { pfun(' '); special--; }
    else if (!super) pfun(' ');
    else { pln(pfun); indent(lm, pfun); }
    superprint(car(form), lm, pfun);
    form = cdr(form);   
  }
  pfun(')'); return;
}

VPtr<object, SPIRAMVAlloc> fn_pprint (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  VPtr<object, SPIRAMVAlloc> obj = first(args);
  pfun_t pfun = pstreamfun(cdr(args));
  pln(pfun);
  superprint(obj, 0, pfun);
  return symbol(NOTHING);
}

VPtr<object, SPIRAMVAlloc> fn_pprintall (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) args, (void) env;
  VPtr<object, SPIRAMVAlloc> globals = GlobalEnv;
  while (globals != nil) {
    VPtr<object, SPIRAMVAlloc> pair = first(globals);
    VPtr<object, SPIRAMVAlloc> var = car(pair);
    VPtr<object, SPIRAMVAlloc> val = cdr(pair);
    pln(pserial);
    if (consp(val) && symbolp(car(val)) && car(val)->val.name == LAMBDA) {
      superprint(cons(symbol(DEFUN), cons(var, cdr(val))), 0, pserial);
    } else {
      superprint(cons(symbol(DEFVAR),cons(var,cons(cons(symbol(QUOTE),cons(val,nil))
      ,nil))), 0, pserial);
    }
    pln(pserial);
    globals = cdr(globals);
  }
  return symbol(NOTHING);
}

// LispLibrary

VPtr<object, SPIRAMVAlloc> fn_require (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  VPtr<object, SPIRAMVAlloc> arg = first(args);
  VPtr<object, SPIRAMVAlloc> globals = GlobalEnv;
  if (!symbolp(arg)) error(REQUIRE, PSTR("argument is not a symbol"), arg);
  while (globals != nil) {
    VPtr<object, SPIRAMVAlloc> pair = first(globals);
    VPtr<object, SPIRAMVAlloc> var = car(pair);
    if (symbolp(var) && var == arg) return nil;
    globals = cdr(globals);
  }
  GlobalStringIndex = 0;
  VPtr<object, SPIRAMVAlloc> line = read(glibrary);
  while (line != nil) {
    // Is this the definition we want
    int fname = first(line)->val.name;
    if ((fname == DEFUN || fname == DEFVAR) && symbolp(second(line)) && second(line)->val.name == arg->val.name) {
      eval(line, env);
      return tee;
    }
    line = read(glibrary);
  }
  return nil; 
}

VPtr<object, SPIRAMVAlloc> fn_listlibrary (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) args, (void) env;
  GlobalStringIndex = 0;
  VPtr<object, SPIRAMVAlloc> line = read(glibrary);
  while (line != nil) {
    int fname = first(line)->val.name;
    if (fname == DEFUN || fname == DEFVAR) {
      pstring(symbolname(second(line)->val.name), pserial); pserial(' ');
    }
    line = read(glibrary);
  }
  return symbol(NOTHING); 
}

// Wi-fi

VPtr<object, SPIRAMVAlloc> fn_available (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  if (isstream(first(args))>>8 != WIFISTREAM) error2(AVAILABLE, PSTR("invalid stream"));
  return number(client.available());
}

VPtr<object, SPIRAMVAlloc> fn_wifiserver (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) args, (void) env;
  server.begin();
  return nil;
}

VPtr<object, SPIRAMVAlloc> fn_wifisoftap (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  char ssid[33], pass[65];
  if (args == nil) return WiFi.softAPdisconnect(true) ? tee : nil;
  VPtr<object, SPIRAMVAlloc> first = first(args); args = cdr(args);
  if (args == nil) WiFi.softAP(cstring(first, ssid, 33));
  else {
    VPtr<object, SPIRAMVAlloc> second = first(args);
    args = cdr(args);
    int channel = 1;
    boolean hidden = false;
    if (args != nil) { 
      channel = checkinteger(WIFISOFTAP, first(args));
      args = cdr(args);
      if (args != nil) hidden = (first(args) != nil);
    }
    WiFi.softAP(cstring(first, ssid, 33), cstring(second, pass, 65), channel, hidden);
  }
  return lispstring((char*)WiFi.softAPIP().toString().c_str());
}

VPtr<object, SPIRAMVAlloc> fn_connected (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  if (isstream(first(args))>>8 != WIFISTREAM) error2(CONNECTED, PSTR("invalid stream"));
  return client.connected() ? tee : nil;
}

VPtr<object, SPIRAMVAlloc> fn_wifilocalip (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) args, (void) env;
  return lispstring((char*)WiFi.localIP().toString().c_str());
}

VPtr<object, SPIRAMVAlloc> fn_wificonnect (VPtr<object, SPIRAMVAlloc> args, VPtr<object, SPIRAMVAlloc> env) {
  (void) env;
  char ssid[33], pass[65];
  if (args == nil) { WiFi.disconnect(true); return nil; }
  if (cdr(args) == nil) WiFi.begin(cstring(first(args), ssid, 33));
  else WiFi.begin(cstring(first(args), ssid, 33), cstring(second(args), pass, 65));
  int result = WiFi.waitForConnectResult();
  if (result == WL_CONNECTED) return lispstring((char*)WiFi.localIP().toString().c_str());
  else if (result == WL_NO_SSID_AVAIL) error2(WIFICONNECT, PSTR("network not found"));
  else if (result == WL_CONNECT_FAILED) error2(WIFICONNECT, PSTR("connection failed"));
  else error2(WIFICONNECT, PSTR("unable to connect"));
  return nil;
}

// Insert your own function definitions here

// Built-in procedure names - stored in PROGMEM

const char string0[] PROGMEM = "nil";
const char string1[] PROGMEM = "t";
const char string2[] PROGMEM = "nothing";
const char string3[] PROGMEM = "&optional";
const char string4[] PROGMEM = "&rest";
const char string5[] PROGMEM = "lambda";
const char string6[] PROGMEM = "let";
const char string7[] PROGMEM = "let*";
const char string8[] PROGMEM = "closure";
const char string9[] PROGMEM = "special_forms";
const char string10[] PROGMEM = "quote";
const char string11[] PROGMEM = "defun";
const char string12[] PROGMEM = "defvar";
const char string13[] PROGMEM = "setq";
const char string14[] PROGMEM = "loop";
const char string15[] PROGMEM = "return";
const char string16[] PROGMEM = "push";
const char string17[] PROGMEM = "pop";
const char string18[] PROGMEM = "incf";
const char string19[] PROGMEM = "decf";
const char string20[] PROGMEM = "setf";
const char string21[] PROGMEM = "dolist";
const char string22[] PROGMEM = "dotimes";
const char string23[] PROGMEM = "trace";
const char string24[] PROGMEM = "untrace";
const char string25[] PROGMEM = "for-millis";
const char string26[] PROGMEM = "with-serial";
const char string27[] PROGMEM = "with-i2c";
const char string28[] PROGMEM = "with-spi";
const char string29[] PROGMEM = "with-sd-card";
const char string29a[] PROGMEM = "with-spiffs";
const char string30[] PROGMEM = "with-client";
const char string31[] PROGMEM = "tail_forms";
const char string32[] PROGMEM = "progn";
const char string33[] PROGMEM = "if";
const char string34[] PROGMEM = "cond";
const char string35[] PROGMEM = "when";
const char string36[] PROGMEM = "unless";
const char string37[] PROGMEM = "case";
const char string38[] PROGMEM = "and";
const char string39[] PROGMEM = "or";
const char string40[] PROGMEM = "functions";
const char string41[] PROGMEM = "not";
const char string42[] PROGMEM = "null";
const char string43[] PROGMEM = "cons";
const char string44[] PROGMEM = "atom";
const char string45[] PROGMEM = "listp";
const char string46[] PROGMEM = "consp";
const char string47[] PROGMEM = "symbolp";
const char string48[] PROGMEM = "streamp";
const char string49[] PROGMEM = "eq";
const char string50[] PROGMEM = "car";
const char string51[] PROGMEM = "first";
const char string52[] PROGMEM = "cdr";
const char string53[] PROGMEM = "rest";
const char string54[] PROGMEM = "caar";
const char string55[] PROGMEM = "cadr";
const char string56[] PROGMEM = "second";
const char string57[] PROGMEM = "cdar";
const char string58[] PROGMEM = "cddr";
const char string59[] PROGMEM = "caaar";
const char string60[] PROGMEM = "caadr";
const char string61[] PROGMEM = "cadar";
const char string62[] PROGMEM = "caddr";
const char string63[] PROGMEM = "third";
const char string64[] PROGMEM = "cdaar";
const char string65[] PROGMEM = "cdadr";
const char string66[] PROGMEM = "cddar";
const char string67[] PROGMEM = "cdddr";
const char string68[] PROGMEM = "length";
const char string69[] PROGMEM = "list";
const char string70[] PROGMEM = "reverse";
const char string71[] PROGMEM = "nth";
const char string72[] PROGMEM = "assoc";
const char string73[] PROGMEM = "member";
const char string74[] PROGMEM = "apply";
const char string75[] PROGMEM = "funcall";
const char string76[] PROGMEM = "append";
const char string77[] PROGMEM = "mapc";
const char string78[] PROGMEM = "mapcar";
const char string79[] PROGMEM = "mapcan";
const char string80[] PROGMEM = "+";
const char string81[] PROGMEM = "-";
const char string82[] PROGMEM = "*";
const char string83[] PROGMEM = "/";
const char string84[] PROGMEM = "mod";
const char string85[] PROGMEM = "1+";
const char string86[] PROGMEM = "1-";
const char string87[] PROGMEM = "abs";
const char string88[] PROGMEM = "random";
const char string89[] PROGMEM = "max";
const char string90[] PROGMEM = "min";
const char string91[] PROGMEM = "/=";
const char string92[] PROGMEM = "=";
const char string93[] PROGMEM = "<";
const char string94[] PROGMEM = "<=";
const char string95[] PROGMEM = ">";
const char string96[] PROGMEM = ">=";
const char string97[] PROGMEM = "plusp";
const char string98[] PROGMEM = "minusp";
const char string99[] PROGMEM = "zerop";
const char string100[] PROGMEM = "oddp";
const char string101[] PROGMEM = "evenp";
const char string102[] PROGMEM = "integerp";
const char string103[] PROGMEM = "numberp";
const char string104[] PROGMEM = "float";
const char string105[] PROGMEM = "floatp";
const char string106[] PROGMEM = "sin";
const char string107[] PROGMEM = "cos";
const char string108[] PROGMEM = "tan";
const char string109[] PROGMEM = "asin";
const char string110[] PROGMEM = "acos";
const char string111[] PROGMEM = "atan";
const char string112[] PROGMEM = "sinh";
const char string113[] PROGMEM = "cosh";
const char string114[] PROGMEM = "tanh";
const char string115[] PROGMEM = "exp";
const char string116[] PROGMEM = "sqrt";
const char string117[] PROGMEM = "log";
const char string118[] PROGMEM = "expt";
const char string119[] PROGMEM = "ceiling";
const char string120[] PROGMEM = "floor";
const char string121[] PROGMEM = "truncate";
const char string122[] PROGMEM = "round";
const char string123[] PROGMEM = "char";
const char string124[] PROGMEM = "char-code";
const char string125[] PROGMEM = "code-char";
const char string126[] PROGMEM = "characterp";
const char string127[] PROGMEM = "stringp";
const char string128[] PROGMEM = "string=";
const char string129[] PROGMEM = "string<";
const char string130[] PROGMEM = "string>";
const char string131[] PROGMEM = "sort";
const char string132[] PROGMEM = "string";
const char string133[] PROGMEM = "concatenate";
const char string134[] PROGMEM = "subseq";
const char string135[] PROGMEM = "read-from-string";
const char string136[] PROGMEM = "princ-to-string";
const char string137[] PROGMEM = "prin1-to-string";
const char string138[] PROGMEM = "logand";
const char string139[] PROGMEM = "logior";
const char string140[] PROGMEM = "logxor";
const char string141[] PROGMEM = "lognot";
const char string142[] PROGMEM = "ash";
const char string143[] PROGMEM = "logbitp";
const char string144[] PROGMEM = "eval";
const char string145[] PROGMEM = "globals";
const char string146[] PROGMEM = "locals";
const char string147[] PROGMEM = "makunbound";
const char string148[] PROGMEM = "break";
const char string149[] PROGMEM = "read";
const char string150[] PROGMEM = "prin1";
const char string151[] PROGMEM = "print";
const char string152[] PROGMEM = "princ";
const char string153[] PROGMEM = "terpri";
const char string154[] PROGMEM = "read-byte";
const char string155[] PROGMEM = "read-line";
const char string156[] PROGMEM = "write-byte";
const char string157[] PROGMEM = "write-string";
const char string158[] PROGMEM = "write-line";
const char string159[] PROGMEM = "restart-i2c";
const char string160[] PROGMEM = "gc";
const char string161[] PROGMEM = "room";
const char string162[] PROGMEM = "save-image";
const char string163[] PROGMEM = "load-image";
const char string164[] PROGMEM = "cls";
const char string165[] PROGMEM = "pinmode";
const char string166[] PROGMEM = "digitalread";
const char string167[] PROGMEM = "digitalwrite";
const char string168[] PROGMEM = "analogread";
const char string169[] PROGMEM = "analogwrite";
const char string170[] PROGMEM = "delay";
const char string171[] PROGMEM = "millis";
const char string172[] PROGMEM = "sleep";
const char string173[] PROGMEM = "note";
const char string174[] PROGMEM = "edit";
const char string175[] PROGMEM = "pprint";
const char string176[] PROGMEM = "pprintall";
const char string177[] PROGMEM = "require";
const char string178[] PROGMEM = "list-library";
const char string179[] PROGMEM = "available";
const char string180[] PROGMEM = "wifi-server";
const char string181[] PROGMEM = "wifi-softap";
const char string182[] PROGMEM = "connected";
const char string183[] PROGMEM = "wifi-localip";
const char string184[] PROGMEM = "wifi-connect";

const tbl_entry_t lookup_table[] PROGMEM = {
  { string0, NULL, 0, 0 },
  { string1, NULL, 0, 0 },
  { string2, NULL, 0, 0 },
  { string3, NULL, 0, 0 },
  { string4, NULL, 0, 0 },
  { string5, NULL, 0, 127 },
  { string6, NULL, 0, 127 },
  { string7, NULL, 0, 127 },
  { string8, NULL, 0, 127 },
  { string9, NULL, NIL, NIL },
  { string10, sp_quote, 1, 1 },
  { string11, sp_defun, 0, 127 },
  { string12, sp_defvar, 2, 2 },
  { string13, sp_setq, 2, 126 },
  { string14, sp_loop, 0, 127 },
  { string15, sp_return, 0, 127 },
  { string16, sp_push, 2, 2 },
  { string17, sp_pop, 1, 1 },
  { string18, sp_incf, 1, 2 },
  { string19, sp_decf, 1, 2 },
  { string20, sp_setf, 2, 126 },
  { string21, sp_dolist, 1, 127 },
  { string22, sp_dotimes, 1, 127 },
  { string23, sp_trace, 0, 1 },
  { string24, sp_untrace, 0, 1 },
  { string25, sp_formillis, 1, 127 },
  { string26, sp_withserial, 1, 127 },
  { string27, sp_withi2c, 1, 127 },
  { string28, sp_withspi, 1, 127 },
  { string29, sp_withsdcard, 2, 127 },
  { string29a, sp_withspiffs, 2, 127 },
  { string30, sp_withclient, 1, 2 },
  { string31, NULL, NIL, NIL },
  { string32, tf_progn, 0, 127 },
  { string33, tf_if, 2, 3 },
  { string34, tf_cond, 0, 127 },
  { string35, tf_when, 1, 127 },
  { string36, tf_unless, 1, 127 },
  { string37, tf_case, 1, 127 },
  { string38, tf_and, 0, 127 },
  { string39, tf_or, 0, 127 },
  { string40, NULL, NIL, NIL },
  { string41, fn_not, 1, 1 },
  { string42, fn_not, 1, 1 },
  { string43, fn_cons, 2, 2 },
  { string44, fn_atom, 1, 1 },
  { string45, fn_listp, 1, 1 },
  { string46, fn_consp, 1, 1 },
  { string47, fn_symbolp, 1, 1 },
  { string48, fn_streamp, 1, 1 },
  { string49, fn_eq, 2, 2 },
  { string50, fn_car, 1, 1 },
  { string51, fn_car, 1, 1 },
  { string52, fn_cdr, 1, 1 },
  { string53, fn_cdr, 1, 1 },
  { string54, fn_caar, 1, 1 },
  { string55, fn_cadr, 1, 1 },
  { string56, fn_cadr, 1, 1 },
  { string57, fn_cdar, 1, 1 },
  { string58, fn_cddr, 1, 1 },
  { string59, fn_caaar, 1, 1 },
  { string60, fn_caadr, 1, 1 },
  { string61, fn_cadar, 1, 1 },
  { string62, fn_caddr, 1, 1 },
  { string63, fn_caddr, 1, 1 },
  { string64, fn_cdaar, 1, 1 },
  { string65, fn_cdadr, 1, 1 },
  { string66, fn_cddar, 1, 1 },
  { string67, fn_cdddr, 1, 1 },
  { string68, fn_length, 1, 1 },
  { string69, fn_list, 0, 127 },
  { string70, fn_reverse, 1, 1 },
  { string71, fn_nth, 2, 2 },
  { string72, fn_assoc, 2, 2 },
  { string73, fn_member, 2, 2 },
  { string74, fn_apply, 2, 127 },
  { string75, fn_funcall, 1, 127 },
  { string76, fn_append, 0, 127 },
  { string77, fn_mapc, 2, 127 },
  { string78, fn_mapcar, 2, 127 },
  { string79, fn_mapcan, 2, 127 },
  { string80, fn_add, 0, 127 },
  { string81, fn_subtract, 1, 127 },
  { string82, fn_multiply, 0, 127 },
  { string83, fn_divide, 1, 127 },
  { string84, fn_mod, 2, 2 },
  { string85, fn_oneplus, 1, 1 },
  { string86, fn_oneminus, 1, 1 },
  { string87, fn_abs, 1, 1 },
  { string88, fn_random, 1, 1 },
  { string89, fn_maxfn, 1, 127 },
  { string90, fn_minfn, 1, 127 },
  { string91, fn_noteq, 1, 127 },
  { string92, fn_numeq, 1, 127 },
  { string93, fn_less, 1, 127 },
  { string94, fn_lesseq, 1, 127 },
  { string95, fn_greater, 1, 127 },
  { string96, fn_greatereq, 1, 127 },
  { string97, fn_plusp, 1, 1 },
  { string98, fn_minusp, 1, 1 },
  { string99, fn_zerop, 1, 1 },
  { string100, fn_oddp, 1, 1 },
  { string101, fn_evenp, 1, 1 },
  { string102, fn_integerp, 1, 1 },
  { string103, fn_numberp, 1, 1 },
  { string104, fn_floatfn, 1, 1 },
  { string105, fn_floatp, 1, 1 },
  { string106, fn_sin, 1, 1 },
  { string107, fn_cos, 1, 1 },
  { string108, fn_tan, 1, 1 },
  { string109, fn_asin, 1, 1 },
  { string110, fn_acos, 1, 1 },
  { string111, fn_atan, 1, 2 },
  { string112, fn_sinh, 1, 1 },
  { string113, fn_cosh, 1, 1 },
  { string114, fn_tanh, 1, 1 },
  { string115, fn_exp, 1, 1 },
  { string116, fn_sqrt, 1, 1 },
  { string117, fn_log, 1, 2 },
  { string118, fn_expt, 2, 2 },
  { string119, fn_ceiling, 1, 2 },
  { string120, fn_floor, 1, 2 },
  { string121, fn_truncate, 1, 2 },
  { string122, fn_round, 1, 2 },
  { string123, fn_char, 2, 2 },
  { string124, fn_charcode, 1, 1 },
  { string125, fn_codechar, 1, 1 },
  { string126, fn_characterp, 1, 1 },
  { string127, fn_stringp, 1, 1 },
  { string128, fn_stringeq, 2, 2 },
  { string129, fn_stringless, 2, 2 },
  { string130, fn_stringgreater, 2, 2 },
  { string131, fn_sort, 2, 2 },
  { string132, fn_stringfn, 1, 1 },
  { string133, fn_concatenate, 1, 127 },
  { string134, fn_subseq, 2, 3 },
  { string135, fn_readfromstring, 1, 1 },
  { string136, fn_princtostring, 1, 1 },
  { string137, fn_prin1tostring, 1, 1 },
  { string138, fn_logand, 0, 127 },
  { string139, fn_logior, 0, 127 },
  { string140, fn_logxor, 0, 127 },
  { string141, fn_lognot, 1, 1 },
  { string142, fn_ash, 2, 2 },
  { string143, fn_logbitp, 2, 2 },
  { string144, fn_eval, 1, 1 },
  { string145, fn_globals, 0, 0 },
  { string146, fn_locals, 0, 0 },
  { string147, fn_makunbound, 1, 1 },
  { string148, fn_break, 0, 0 },
  { string149, fn_read, 0, 1 },
  { string150, fn_prin1, 1, 2 },
  { string151, fn_print, 1, 2 },
  { string152, fn_princ, 1, 2 },
  { string153, fn_terpri, 0, 1 },
  { string154, fn_readbyte, 0, 2 },
  { string155, fn_readline, 0, 1 },
  { string156, fn_writebyte, 1, 2 },
  { string157, fn_writestring, 1, 2 },
  { string158, fn_writeline, 1, 2 },
  { string159, fn_restarti2c, 1, 2 },
  { string160, fn_gc, 0, 0 },
  { string161, fn_room, 0, 0 },
  { string162, fn_saveimage, 0, 1 },
  { string163, fn_loadimage, 0, 1 },
  { string164, fn_cls, 0, 0 },
  { string165, fn_pinmode, 2, 2 },
  { string166, fn_digitalread, 1, 1 },
  { string167, fn_digitalwrite, 2, 2 },
  { string168, fn_analogread, 1, 1 },
  { string169, fn_analogwrite, 2, 2 },
  { string170, fn_delay, 1, 1 },
  { string171, fn_millis, 0, 0 },
  { string172, fn_sleep, 1, 1 },
  { string173, fn_note, 0, 3 },
  { string174, fn_edit, 1, 1 },
  { string175, fn_pprint, 1, 2 },
  { string176, fn_pprintall, 0, 0 },
  { string177, fn_require, 1, 1 },
  { string178, fn_listlibrary, 0, 0 },
  { string179, fn_available, 1, 1 },
  { string180, fn_wifiserver, 0, 0 },
  { string181, fn_wifisoftap, 0, 4 },
  { string182, fn_connected, 1, 1 },
  { string183, fn_wifilocalip, 0, 0 },
  { string184, fn_wificonnect, 0, 2 },
};

// Table lookup functions

int builtin (char* n) {
  int entry = 0;
  while (entry < ENDFUNCTIONS) {
    if (strcasecmp(n, (char*)lookup_table[entry].string) == 0)
      return entry;
    entry++;
  }
  return ENDFUNCTIONS;
}

int longsymbol (char *buffer) {
  char *p = SymbolTable;
  int i = 0;
  while (strcasecmp(p, buffer) != 0) {p = p + strlen(p) + 1; i++; }
  if (p == buffer) {
    // Add to symbol table?
    char *newtop = SymbolTop + strlen(p) + 1;
    if (SYMBOLTABLESIZE - (newtop - SymbolTable) < BUFFERSIZE) error2(0, PSTR("no room for long symbols"));
    SymbolTop = newtop;
  }
  if (i > 1535) error2(0, PSTR("Too many long symbols"));
  return i + 64000; // First number unused by radix40
}

intptr_t lookupfn (symbol_t name) {
  return (intptr_t)lookup_table[name].fptr;
}

uint8_t lookupmin (symbol_t name) {
  return lookup_table[name].min;
}

uint8_t lookupmax (symbol_t name) {
  return lookup_table[name].max;
}

char *lookupbuiltin (symbol_t name) {
  char *buffer = SymbolTop;
  strcpy(buffer, (char *)lookup_table[name].string);
  return buffer;
}

char *lookupsymbol (symbol_t name) {
  char *p = SymbolTable;
  int i = name - 64000;
  while (i > 0 && p < SymbolTop) {p = p + strlen(p) + 1; i--; }
  if (p == SymbolTop) return NULL; else return p;
}

void deletesymbol (symbol_t name) {
  char *p = lookupsymbol(name);
  if (p == NULL) return;
  char *q = p + strlen(p) + 1;
  *p = '\0'; p++;
  while (q < SymbolTop) *(p++) = *(q++);
  SymbolTop = p;
}

void testescape () {
  if (Serial.read() == '~') error2(0, PSTR("escape!"));
}

// Main evaluator

uint8_t End;

VPtr<object, SPIRAMVAlloc> eval (VPtr<object, SPIRAMVAlloc> form, VPtr<object, SPIRAMVAlloc> env) {
  int TC=0;
  EVAL:
  yield(); // Needed on ESP8266 to avoid Soft WDT Reset
  // Enough space?
  if (End != 0xA5) error2(0, PSTR("Stack overflow"));
  if (Freespace <= (WORKSPACESIZE)>>4) gc(form, env);
  // Escape
  if (tstflag(ESCAPE)) { clrflag(ESCAPE); error2(0, PSTR("Escape!"));}
  #if defined (serialmonitor)
  if (!tstflag(NOESC)) testescape();
  #endif
  
  if (form == nil) return nil;

  if (integerp(form) || floatp(form) || characterp(form) || stringp(form)) return form;

  if (symbolp(form)) {
    symbol_t name = form->val.name;
    if (name == NIL) return nil;
    VPtr<object, SPIRAMVAlloc> pair = value(name, env);
    if (pair != nil) return cdr(pair);
    pair = value(name, GlobalEnv);
    if (pair != nil) return cdr(pair);
    else if (name <= ENDFUNCTIONS) return form;
    error(0, PSTR("undefined"), form);
  }
  
  // It's a list
  VPtr<object, SPIRAMVAlloc> function = car(form);
  VPtr<object, SPIRAMVAlloc> args = cdr(form);

  if (function == nil) error(0, PSTR("illegal function"), nil);
  if (!listp(args)) error(0, PSTR("can't evaluate a dotted pair"), args);

  // List starts with a symbol?
  if (symbolp(function)) {
    symbol_t name = function->val.name;

    if ((name == LET) || (name == LETSTAR)) {
      int TCstart = TC;
      VPtr<object, SPIRAMVAlloc> assigns = first(args);
      if (!listp(assigns)) error(name, PSTR("first argument is not a list"), assigns);
      VPtr<object, SPIRAMVAlloc> forms = cdr(args);
      VPtr<object, SPIRAMVAlloc> newenv = env;
      push(newenv, GCStack);
      while (assigns != nil) {
        VPtr<object, SPIRAMVAlloc> assign = car(assigns);
        if (!consp(assign)) push(cons(assign,nil), newenv);
        else if (cdr(assign) == nil) push(cons(first(assign),nil), newenv);
        else push(cons(first(assign),eval(second(assign),env)), newenv);
        car(GCStack) = newenv;
        if (name == LETSTAR) env = newenv;
        assigns = cdr(assigns);
      }
      env = newenv;
      pop(GCStack);
      form = tf_progn(forms,env);
      TC = TCstart;
      goto EVAL;
    }

    if (name == LAMBDA) {
      if (env == nil) return form;
      VPtr<object, SPIRAMVAlloc> envcopy = nil;
      while (env != nil) {
        VPtr<object, SPIRAMVAlloc> pair = first(env);
        if (pair != nil) push(pair, envcopy);
        env = cdr(env);
      }
      return cons(symbol(CLOSURE), cons(envcopy,args));
    }
    
    if (name < SPECIAL_FORMS) error2((int)function.getRawNum(), PSTR("can't be used as a function"));

    if ((name > SPECIAL_FORMS) && (name < TAIL_FORMS)) {
      return ((fn_ptr_type)lookupfn(name))(args, env);
    }

    if ((name > TAIL_FORMS) && (name < FUNCTIONS)) {
      form = ((fn_ptr_type)lookupfn(name))(args, env);
      TC = 1;
      goto EVAL;
    }
  }
        
  // Evaluate the parameters - result in head
  VPtr<object, SPIRAMVAlloc> fname = car(form);
  int TCstart = TC;
  VPtr<object, SPIRAMVAlloc> head = cons(eval(car(form), env), nil);
  push(head, GCStack); // Don't GC the result list
  VPtr<object, SPIRAMVAlloc> tail = head;
  form = cdr(form);
  int nargs = 0;

  while (form != nil){
    VPtr<object, SPIRAMVAlloc> obj = cons(eval(car(form),env),nil);
    cdr(tail) = obj;
    tail = obj;
    form = cdr(form);
    nargs++;
  }
    
  function = car(head);
  args = cdr(head);
 
  if (symbolp(function)) {
    symbol_t name = function->val.name;
    if (name >= ENDFUNCTIONS) error(0, PSTR("not valid here"), fname);
    if (nargs<lookupmin(name)) error2(name, PSTR("has too few arguments"));
    if (nargs>lookupmax(name)) error2(name, PSTR("has too many arguments"));
    VPtr<object, SPIRAMVAlloc> result = ((fn_ptr_type)lookupfn(name))(args, env);
    pop(GCStack);
    return result;
  }
      
  if (consp(function) && issymbol(car(function), LAMBDA)) {
    form = closure(TCstart, fname->val.name, nil, cdr(function), args, &env);
    pop(GCStack);
    int trace = tracing(fname->val.name);
    if (trace) {
      VPtr<object, SPIRAMVAlloc> result = eval(form, env);
      indent((--(TraceDepth[trace-1]))<<1, pserial);
      pint(TraceDepth[trace-1], pserial);
      pserial(':'); pserial(' ');
      printobject(fname, pserial); pfstring(PSTR(" returned "), pserial);
      printobject(result, pserial); pln(pserial);
      return result;
    } else {
      TC = 1;
      goto EVAL;
    }
  }

  if (consp(function) && issymbol(car(function), CLOSURE)) {
    function = cdr(function);
    form = closure(TCstart, fname->val.name, car(function), cdr(function), args, &env);
    pop(GCStack);
    TC = 1;
    goto EVAL;
  } 
  
  error(0, PSTR("illegal function"), fname); return nil;
}

// Print functions

inline int maxbuffer (char *buffer) {
  return SYMBOLTABLESIZE-(buffer-SymbolTable)-1;
}

void pserial (char c) {
  LastPrint = c;
  if (c == '\n') Serial.write('\r');
  Serial.write(c);
}

const char ControlCodes[] PROGMEM = "Null\0SOH\0STX\0ETX\0EOT\0ENQ\0ACK\0Bell\0Backspace\0Tab\0Newline\0VT\0"
"Page\0Return\0SO\0SI\0DLE\0DC1\0DC2\0DC3\0DC4\0NAK\0SYN\0ETB\0CAN\0EM\0SUB\0Escape\0FS\0GS\0RS\0US\0Space\0";

void pcharacter (char c, pfun_t pfun) {
  if (!tstflag(PRINTREADABLY)) pfun(c);
  else {
    pfun('#'); pfun('\\');
    if (c > 32) pfun(c);
    else {
      const char *p = ControlCodes;
      while (c > 0) {p = p + strlen(p) + 1; c--; }
      pfstring(p, pfun);
    }
  }
}

void pstring (char *s, pfun_t pfun) {
  while (*s) pfun(*s++);
}

void printstring (VPtr<object, SPIRAMVAlloc> form, pfun_t pfun) {
  if (tstflag(PRINTREADABLY)) pfun('"');
  form = cdr(form);
  while (form != nil) {
    int chars = form->val.integer;
    for (int i=(sizeof(int)-1)*8; i>=0; i=i-8) {
      char ch = chars>>i & 0xFF;
      if (tstflag(PRINTREADABLY) && (ch == '"' || ch == '\\')) pfun('\\');
      if (ch) pfun(ch);
    }
    form = car(form);
  }
  if (tstflag(PRINTREADABLY)) pfun('"');
}

void pfstring (const char *s, pfun_t pfun) {
  int p = 0;
  while (1) {
    char c = s[p++];
    if (c == 0) return;
    pfun(c);
  }
}

void pint (int i, pfun_t pfun) {
  int lead = 0;
  #if INT_MAX == 32767
  int p = 10000;
  #else
  int p = 1000000000;
  #endif
  if (i<0) pfun('-');
  for (int d=p; d>0; d=d/10) {
    int j = i/d;
    if (j!=0 || lead || d==1) { pfun(abs(j)+'0'); lead=1;}
    i = i - j*d;
  }
}

void pmantissa (float f, pfun_t pfun) {
  int sig = floor(log10(f));
  int mul = pow(10, 5 - sig);
  int i = round(f * mul);
  boolean point = false;
  if (i == 1000000) { i = 100000; sig++; }
  if (sig < 0) {
    pfun('0'); pfun('.'); point = true;
    for (int j=0; j < - sig - 1; j++) pfun('0');
  }
  mul = 100000;
  for (int j=0; j<7; j++) {
    int d = (int)(i / mul);
    pfun(d + '0');
    i = i - d * mul;
    if (i == 0) { 
      if (!point) {
        for (int k=j; k<sig; k++) pfun('0');
        pfun('.'); pfun('0');
      }
      return;
    }
    if (j == sig && sig >= 0) { pfun('.'); point = true; }
    mul = mul / 10;
  }
}

void pfloat (float f, pfun_t pfun) {
  if (isnan(f)) { pfstring(PSTR("NaN"), pfun); return; }
  if (f == 0.0) { pfun('0'); return; }
  if (isinf(f)) { pfstring(PSTR("Inf"), pfun); return; }
  if (f < 0) { pfun('-'); f = -f; }
  // Calculate exponent
  int e = 0;
  if (f < 1e-3 || f >= 1e5) {
    e = floor(log(f) / 2.302585); // log10 gives wrong result
    f = f / pow(10, e);
  }
  
  pmantissa (f, pfun);
  
  // Exponent
  if (e != 0) {
    pfun('e');
    pint(e, pfun);
  }
}

inline void pln (pfun_t pfun) {
  pfun('\n');
}

void pfl (pfun_t pfun) {
  if (LastPrint != '\n') pfun('\n');
}

void printobject (VPtr<object, SPIRAMVAlloc> form, pfun_t pfun){
  if (form == nil) pfstring(PSTR("nil"), pfun);
  else if (listp(form) && issymbol(car(form), CLOSURE)) pfstring(PSTR("<closure>"), pfun);
  else if (listp(form)) {
    pfun('(');
    printobject(car(form), pfun);
    form = cdr(form);
    while (form != nil && listp(form)) {
      pfun(' ');
      printobject(car(form), pfun);
      form = cdr(form);
    }
    if (form != nil) {
      pfstring(PSTR(" . "), pfun);
      printobject(form, pfun);
    }
    pfun(')');
  } else if (integerp(form)) pint(form->val.integer, pfun);
  else if (floatp(form)) pfloat(form->val.single_float, pfun);
  else if (symbolp(form)) { if (form->val.name != NOTHING) pstring(symbolname(form->val.name), pfun); }
  else if (characterp(form)) pcharacter(form->val.integer, pfun);
  else if (stringp(form)) printstring(form, pfun);
  else if (streamp(form)) {
    pfstring(PSTR("<"), pfun);
    if ((form->val.integer)>>8 == SPISTREAM) pfstring(PSTR("spi"), pfun);
    else if ((form->val.integer)>>8 == I2CSTREAM) pfstring(PSTR("i2c"), pfun);
    else if ((form->val.integer)>>8 == SDSTREAM) pfstring(PSTR("sd"), pfun);
    else pfstring(PSTR("serial"), pfun);
    pfstring(PSTR("-stream "), pfun);
    pint(form->val.integer & 0xFF, pfun);
    pfun('>');
  } else
    error2(0, PSTR("Error in print"));
}

// Read functions

int glibrary () {
  if (LastChar) { 
    char temp = LastChar;
    LastChar = 0;
    return temp;
  }
  char c = LispLibrary[GlobalStringIndex++];
  return (c != 0) ? c : -1; // -1?
}

void loadfromlibrary (VPtr<object, SPIRAMVAlloc> env) {   
  GlobalStringIndex = 0;
  VPtr<object, SPIRAMVAlloc> line = read(glibrary);
  while (line != nil) {
    eval(line, env);
    line = read(glibrary);
  }
}

int gserial () {
  if (LastChar) { 
    char temp = LastChar;
    LastChar = 0;
    return temp;
  }
  while (!Serial.available());
  char temp = Serial.read();
  if (temp != '\n') pserial(temp);
  return temp;
}

VPtr<object, SPIRAMVAlloc> nextitem (gfun_t gfun) {
  int ch = gfun();
  while(isspace(ch)) ch = gfun();

  if (ch == ';') {
    while(ch != '(') ch = gfun();
    ch = '(';
  }
  if (ch == '\n') ch = gfun();
  if (ch == -1) return nil;
  if (ch == ')') return  KETToken;
  if (ch == '(') return BRAToken;
  if (ch == '\'') return QUOToken;

  // Parse string
  if (ch == '"') return readstring('"', gfun);
  
  // Parse symbol, character, or number
  int index = 0, base = 10, sign = 1;
  char *buffer = SymbolTop;
  int bufmax = maxbuffer(buffer); // Max index
  unsigned int result = 0;
  boolean isfloat = false;
  float fresult = 0.0;

  if (ch == '+') {
    buffer[index++] = ch;
    ch = gfun();
  } else if (ch == '-') {
    sign = -1;
    buffer[index++] = ch;
    ch = gfun();
  } else if (ch == '.') {
    buffer[index++] = ch;
    ch = gfun();
    if (ch == ' ') return DOTToken;
    isfloat = true;
  } else if (ch == '#') {
    ch = gfun();
    char ch2 = ch & ~0x20; // force to upper case
    if (ch == '\\') base = 0; // character
    else if (ch2 == 'B') base = 2;
    else if (ch2 == 'O') base = 8;
    else if (ch2 == 'X') base = 16;
    else if (ch == '\'') return nextitem(gfun);
    else if (ch == '.') {
      setflag(NOESC);
      VPtr<object, SPIRAMVAlloc> result = eval(read(gfun), nil);
      clrflag(NOESC);
      return result;
    } else error2(0, PSTR("illegal character after #"));
    ch = gfun();
  }
  int valid; // 0=undecided, -1=invalid, +1=valid
  if (ch == '.') valid = 0; else if (digitvalue(ch)<base) valid = 1; else valid = -1;
  boolean isexponent = false;
  int exponent = 0, esign = 1;
  buffer[2] = '\0'; // In case symbol is one letter
  float divisor = 10.0;
  
  while(!isspace(ch) && ch != ')' && ch != '(' && index < bufmax) {
    buffer[index++] = ch;
    if (base == 10 && ch == '.' && !isexponent) {
      isfloat = true;
      fresult = result;
    } else if (base == 10 && (ch == 'e' || ch == 'E')) {
      if (!isfloat) { isfloat = true; fresult = result; }
      isexponent = true;
      if (valid == 1) valid = 0; else valid = -1;
    } else if (isexponent && ch == '-') {
      esign = -esign;
    } else if (isexponent && ch == '+') {
    } else {
      int digit = digitvalue(ch);
      if (digitvalue(ch)<base && valid != -1) valid = 1; else valid = -1;
      if (isexponent) {
        exponent = exponent * 10 + digit;
      } else if (isfloat) {
        fresult = fresult + digit / divisor;
        divisor = divisor * 10.0;
      } else {
        result = result * base + digit;
      }
    }
    ch = gfun();
  }

  buffer[index] = '\0';
  if (ch == ')' || ch == '(') LastChar = ch;
  if (isfloat && valid == 1) return makefloat(fresult * sign * pow(10, exponent * esign));
  else if (valid == 1) {
    if (base == 10 && result > ((unsigned int)INT_MAX+(1-sign)/2)) 
      return makefloat((float)result*sign);
    return number(result*sign);
  } else if (base == 0) {
    if (index == 1) return character(buffer[0]);
    const char* p = ControlCodes; char c = 0;
    while (c < 33) {
      if (strcasecmp(buffer, p) == 0) return character(c);
      p = p + strlen(p) + 1; c++;
    }
    error2(0, PSTR("Unknown character"));
  }
  
  int x = builtin(buffer);
  if (x == NIL) return nil;
  if (x < ENDFUNCTIONS) return newsymbol(x);
  else if (index < 4 && valid40(buffer)) return newsymbol(pack40(buffer));
  else return newsymbol(longsymbol(buffer));
}

VPtr<object, SPIRAMVAlloc> readrest (gfun_t gfun) {
  VPtr<object, SPIRAMVAlloc> item = nextitem(gfun);
  VPtr<object, SPIRAMVAlloc> head = nil;
  VPtr<object, SPIRAMVAlloc> tail = nil;

  while (item.getRawNum() != KET) {
    if (item.getRawNum() == BRA) {
      item = readrest(gfun);
    } else if (item.getRawNum() == QUO) {
      item = cons(symbol(QUOTE), cons(read(gfun), nil));
    } else if (item.getRawNum() == DOT) {
      tail->ptr.cdr = read(gfun);
      if (readrest(gfun) != nil) error2(0, PSTR("malformed list"));
      return head;
    } else {
      VPtr<object, SPIRAMVAlloc> cell = cons(item, nil);
      if (head == nil) head = cell;
      else tail->ptr.cdr = cell;
      tail = cell;
      item = nextitem(gfun);
    }
  }
  return head;
}

VPtr<object, SPIRAMVAlloc> read (gfun_t gfun) {
  VPtr<object, SPIRAMVAlloc> item = nextitem(gfun);
  if (item.getRawNum() == KET) error2(0, PSTR("incomplete list"));
  if (item.getRawNum() == BRA) return readrest(gfun);
  if (item.getRawNum() == DOT) return read(gfun);
  if (item.getRawNum() == QUO) return cons(symbol(QUOTE), cons(read(gfun), nil)); 
  return item;
}

// Setup

void initenv () {
  GlobalEnv = nil;
  tee = symbol(TEE);
}

void setup () {
  Serial.begin(9600);
  int start = millis();
  while ((millis() - start) < 5000) { if (Serial) break; }
  valloc.start();
  initworkspace();
  initenv();
  initsleep();
  pfstring(PSTR("uLisp 3.0 "), pserial); pln(pserial);

  BRAToken.setRawNum(BRA);
  KETToken.setRawNum(KET);
  QUOToken.setRawNum(QUO);
  DOTToken.setRawNum(DOT);
}

// Read/Evaluate/Print loop

void repl (VPtr<object, SPIRAMVAlloc> env) {
  for (;;) {
    randomSeed(micros());
    gc(nil, env);
    #if defined (printfreespace)
    pint(Freespace, pserial);
    #endif
    if (BreakLevel) {
      pfstring(PSTR(" : "), pserial);
      pint(BreakLevel, pserial);
    }
    pfstring(PSTR("> "), pserial);
    VPtr<object, SPIRAMVAlloc> line = read(gserial);
    if (BreakLevel && line == nil) { pln(pserial); return; }
    if (line.getRawNum() == KET) error2(0, PSTR("unmatched right bracket"));
    push(line, GCStack);
    pfl(pserial);
    line = eval(line, env);
    pfl(pserial);
    printobject(line, pserial);
    pop(GCStack);
    pfl(pserial);
    pln(pserial);
  }
}

void loop () {
  End = 0xA5;      // Canary to check stack
  if (!setjmp(exception)) {
    #if defined(resetautorun)
    volatile int autorun = 12; // Fudge to keep code size the same
    #else
    volatile int autorun = 13;
    #endif
    if (autorun == 12) autorunimage();
  }
  // Come here after error
  delay(100); while (Serial.available()) Serial.read();
  for (int i=0; i<TRACEMAX; i++) TraceDepth[i] = 0;
  #if defined(sdcardsupport)
  SDpfile.close(); SDgfile.close();
  #endif
  SPIFFSpfile.close(); SPIFFSgfile.close();
  #if defined(lisplibrary)
  if (!tstflag(LIBRARYLOADED)) { setflag(LIBRARYLOADED); loadfromlibrary(nil); }
  #endif
  client.stop();
  repl(nil);
  valloc.stop();
}
