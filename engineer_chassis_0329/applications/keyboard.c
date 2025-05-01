#include "main.h"
#include "remote_control.h"
#include "keyboard.h"

all_key_t all_key;

void key_init(key_t *key_init, key_e KEY);
void key_itself_press_num(key_t *press_num, int8_t num);
int8_t key_press(key_t *press);

const RC_ctrl_t *key_RC;

void key_init(key_t *key_init, key_e KEY)
{
	key_init->content = KEY;
	key_init->itself.flag = 0;
	key_init->itself.mode = 0;
	key_init->itself.time = 0;
	key_init->itself.last_mode = 0;
}

void key_itself_press_num(key_t *press_num, int8_t num)
{
	press_num->itself.last_mode = press_num->itself.mode;
	if(press_num->itself.flag == 0)                      
	{
			if(key_press(press_num))              
			{
					press_num->itself.time++;
			}
			if(press_num->itself.time >= 10) 
			{	
					press_num->itself.flag = 1;
					press_num->itself.time = 0;
			}
	}
	else                                        
	{
			if(!(key_press(press_num)))   
			{
					press_num->itself.time++;
			}
			if(press_num->itself.time >= 10) 
			{	
					press_num->itself.flag = 0;
					press_num->itself.time = 0;
					press_num->itself.mode = press_num->itself.mode +1;
				if(press_num->itself.mode == num)
					press_num->itself.mode=0;
			}
	}
}


int8_t key_press(key_t *press)
{
		key_RC = get_remote_control_point();
		switch(press->content)
		{
			case W:
				{
					if(key_RC->key.W)
						return 1;
					else
						return 0;
				}
				case S:
				{
					if(key_RC->key.S)
						return 1;
					else
						return 0;
				}
				case A:
				{
					if(key_RC->key.A)
						return 1;
					else
						return 0;
				}
				case D:
				{
					if(key_RC->key.D)
						return 1;
					else
						return 0;
				}
				case SHIFT:
				{
					if(key_RC->key.SHIFT)
						return 1;
					else
						return 0;
				}
				case CTRL:
				{
					if(key_RC->key.CTRL)
						return 1;
					else
						return 0;
				}
				case Q:
				{
					if(key_RC->key.Q)
						return 1;
					else
						return 0;
				}
				case E:
				{
					if(key_RC->key.E)
						return 1;
					else
						return 0;
				}
				case R:
				{
					if(key_RC->key.R)
						return 1;
					else
						return 0;
				}
				case F:
				{
					if(key_RC->key.F)
						return 1;
					else
						return 0;
				}
				case G:
				{
					if(key_RC->key.G)
						return 1;
					else
						return 0;
				}
				case Z:
				{
					if(key_RC->key.Z)
						return 1;
					else
						return 0;
				}
				case X:
				{
					if(key_RC->key.X)
						return 1;
					else
						return 0;
				}
				case C:
				{
					if(key_RC->key.C)
						return 1;
					else
						return 0;
				}
				case V:
				{
					if(key_RC->key.V)
						return 1;
					else
						return 0;
				}
				case B:
				{
					if(key_RC->key.B)
						return 1;
					else
						return 0;
				}
				case PL:
				{
					if(key_RC->mouse.press_l)
						return 1;
					else
						return 0;
				}
				case PR:
				{
					if(key_RC->mouse.press_r)
						return 1;
					else
						return 0;
				}

				default:
					break;
				
		}
		return 0;
}
