#ifndef KEY_H
#define KEY_H
#include "struct_typedef.h"

typedef enum
{
		W,
		S,
		A,
		D,
		PR,
		PL,
		F,
}key_e;

typedef struct
{
	int8_t mode;//按一次加1
	int8_t time;
	int8_t flag;//按下/松开状态指示
	int8_t last_mode;
}key_mode_t;

typedef struct
{
	key_e content;
	key_mode_t itself;
}key_t;

typedef struct
{
	key_t yaw_plus_key;
	key_t yaw_minus_key;
	key_t clamp_key;
}all_key_t;

extern all_key_t all_key;

extern void key_init(key_t *key_init, key_e KEY);
extern void key_itself_press_num(key_t *press_num, int8_t num);  
extern int8_t key_press(key_t *press);     
#endif
