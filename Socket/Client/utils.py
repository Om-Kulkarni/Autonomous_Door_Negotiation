from types import Input


def input_to_message(input: Input):
    if isinstance(input, list):
        if len(input) == 0:
            return []
        if all(isinstance(i, int) for i in input):
            return input
