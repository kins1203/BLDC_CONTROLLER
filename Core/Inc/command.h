#ifndef __COMMAND_H
#define __COMMAND_H

#include <stdint.h>

typedef struct
{
    char cmd[16];      // lệnh chính
    char key[16];      // tham số (nếu có)
    float value;       // giá trị (nếu có)
    uint8_t has_key;
    uint8_t has_value;
} CmdParsed_t;

/**
 * @brief  Parse command dạng:
 *         CMD
 *         CMD, KEY=VALUE
 *
 * @param  input  chuỗi đầu vào (sẽ bị chỉnh sửa nội dung)
 * @param  out    struct kết quả parse
 *
 * @retval 1 nếu parse thành công
 *         0 nếu lỗi
 */
int Command_Parse(char *input, CmdParsed_t *out);

#endif /* __COMMAND_H */
