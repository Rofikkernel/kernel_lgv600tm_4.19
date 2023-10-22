#ifndef _HALL_IC_H
#define _HALL_IC_H

#if defined(CONFIG_LGE_DUAL_SCREEN)
#define FRONTCOVER_CLOSE 1
#define BACKCOVER_CLOSE 5
#endif
struct hallic_dev {
	const char	*name;
	struct device	*dev;
	int 		state;
	int		state_front;
	int		state_back;
};

int hallic_register(struct hallic_dev *hdev);
void hallic_unregister(struct hallic_dev *hdev);

static inline int hallic_get_state(struct hallic_dev *hdev)
{ return hdev->state; }
void hallic_set_state(struct hallic_dev *hdev, int state);

#if defined(CONFIG_LGE_DUAL_SCREEN)
extern struct hallic_dev luke_sdev;
#endif
#if defined(CONFIG_LGE_SUPPORT_HALLIC)
extern struct hallic_dev sdev;
#endif
#endif
