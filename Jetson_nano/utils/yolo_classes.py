"""yolo_classes.py
"""



SIGN_CLASSES = [

    'turn_left',
    'turn_right',
    'speed_50',
    'speed_20',
    'stop',
    'go_ahead',

]



def get_cls_dict(category_num):
    """Get the class ID to name translation dictionary."""
    if category_num == 6:
        return {i: n for i, n in enumerate(SIGN_CLASSES)}
    else:
        return {i: 'CLS%d' % i for i in range(category_num)}
