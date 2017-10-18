from what_the_duck.mongo_suppor import get_upload_collection
from collections import defaultdict
from duckietown_utils import logger
from duckietown_utils import yaml_dump
from duckietown_utils import write_data_to_file

def get_valid_data(collection):
    
    res = list(collection.find())
    out = []
    for r in res:
        if 'what_the_duck_version' in r:
            if r['what_the_duck_version'] in ['1.1']:
                del r['_id']
                out.append(r)
    
    logger.info('Found %d database entries; %d are compatible.' % 
                (len(res), len(out)))
    return out
    
    
def what_the_duck_stats():
    collection = get_upload_collection()
    
    res = list(get_valid_data(collection))

#     for r in res:
#         del r['_id']
#          
    data = yaml_dump(res)
        
    write_data_to_file(data, 'last_download.yaml')
        
    hostnames = defaultdict(lambda:0)
    
    for r in res:
        hostnames[r['hostname']] += 1

    s = 'Number of tests by hostname: '
    for h, n in hostnames.items():
        s += '\n- %s: %d tests' % (h, n)
    
    logger.info(s)
    
    