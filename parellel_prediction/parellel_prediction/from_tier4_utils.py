import unique_identifier_msgs.msg._uuid as uuid



class Tier4Utils():
    '''Methods for map_based_prediction_node.'''
    def __init__(self):
        print('Tier4Utils class is ready!')
    

    # Methods:
    def toHexString(self, id:uuid.UUID) -> str:
        hex_string = ""
        for i in range(16):
            hex_string += format(id.uuid[i], '02x')
        return hex_string



def main():
    pass


if __name__ == '__main__':
    main()