# combine all tutorial documentation
import os

file_list = ['README.md',
             'getting_started.md',
             'drone_control.md',
             'old_school.md',
             'simple_class.md',
             'modular.md',
             'finite_state.md',
             'ros_timer.md',
             'perception.md',
             'starling.md']

#file_list = ['simple_class.md']

lines = {}
tags = {}

for file_name in file_list:
    f = open('tutorial/'+file_name, "r")
    lines[file_name] = f.readlines()
    tags[file_name] = lines[file_name][0]
    assert(tags[file_name].startswith('# '))
    tags[file_name] = tags[file_name][2:].rstrip().lower().replace(' ','-')

#print(tags)

#print(lines['simple_class.md'])

with open('tutorial/doc.md','w') as f:
    for file_name in file_list:
        for ln in lines[file_name]:
            for targ_file in file_list:
                #print('replacing '+'({})'.format(targ_file))
                #print('with '+'(#{})'.format(tags[targ_file]))
                ln = ln.replace('({})'.format(targ_file),'(#{})'.format(tags[targ_file]))
            ln = ln.replace('[Back to tutorial contents](README.md#contents)','[Back to top](#{})'.format(tags['README.md']))
            ln = ln.replace('(../','(https://github.com/StarlingUAS/fenswood_volcano_template/tree/main/')
            #print(ln)
            f.write(ln)
        f.write('\n')