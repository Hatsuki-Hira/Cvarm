def find_white_line(line):

    max_length = 0
    current_length = 0

    start = 0

    best_left = 0
    best_right = 0

    for x in range(len(line)):
        pixel = line[x]
        if pixel == '1':
            # 新白线的开始端
            if current_length == 0:
                start = x

            current_length += 1

            if current_length > max_length:
                max_length = current_length

                best_left = start
                best_right = x

        else:
            current_length = 0

    print(best_left, best_right)

find_white_line("1001111001")