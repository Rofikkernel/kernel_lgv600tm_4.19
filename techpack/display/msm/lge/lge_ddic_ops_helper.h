#ifndef _LGE_DDIC_OPS_HELPER_H_
#define _LGE_DDIC_OPS_HELPER_H_

extern int store_aod_area(struct dsi_panel *panel, struct lge_rect rect);
extern struct dsi_cmd_desc* find_cmd(struct dsi_cmd_desc *cmds, int cmds_count, int addr);
extern struct dsi_cmd_desc* find_nth_cmd(struct dsi_cmd_desc *cmds, int cmds_count, int addr, int nth);
extern bool is_need_register_backup(struct dsi_panel *panel);
extern int lge_ddic_dsi_panel_reg_backup(struct dsi_panel *panel);
extern int lge_ddic_dsi_panel_reg_backup_reinit(struct dsi_panel *panel);
extern bool is_bist_supported(struct dsi_panel *panel, char *type);

#define WORD_UPPER_BYTE(w) (((w)>>8)&0xff)
#define WORD_LOWER_BYTE(w) ((w)&0xff)
#define WORDS_TO_BYTE_ARRAY(w1, w2, b) do {\
		b[0] = WORD_UPPER_BYTE(w1);\
		b[1] = WORD_LOWER_BYTE(w1);\
		b[2] = WORD_UPPER_BYTE(w2);\
		b[3] = WORD_LOWER_BYTE(w2);\
} while(0)

#endif // _LGE_DDIC_OPS_HELPER_H_
