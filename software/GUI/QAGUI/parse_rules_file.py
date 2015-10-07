import sys






def parseProfile(str):
    profile = str.strip("\n")
    profile = profile.lstrip('<').rstrip('>')
    profile = profile.replace(" ","").split(',')
    return profile

def parseText(str):
    #removes unncessary spaces and the text quotes (")
    text = str.replace("\"","").strip(" \n")
    return text

def parseRulesFile(file):
    print "Parsing file: %s\n" % file.name
    list_of_rules = []
    for line in file:
        if line[0] != '#' and line[0] != '\n': #comment or empty line
            myList = line.split(':')
            if len(myList) != 2:
                print "wrong format ... skipped"
            else:
                profile = parseProfile(myList[0])
                text_to_say = parseText(myList[1])
                rule = (profile , text_to_say)
                list_of_rules.append(rule)

    return list_of_rules


rules_filename = sys.argv[1]
try:
    f = open(rules_filename, 'r')
except IOError:
    print 'cannot open', rules_filename
else:
    list_of_rules = parseRulesFile(f)
    print "LIST OF RULES"
    for rule in list_of_rules: print(rule)    

    f.close()


instance_filename = sys.argv[2]
try:
    f = open(instance_filename, 'r')
except IOError:
    print 'cannot open', instance_filename
else:
    line = f.readline()
    f.close()

    profile = parseProfile(line)
    print "INPUT PROFILE", profile

    #look for more suitable rule
    candidate_profiles = []
    for rule in list_of_rules:
        candidate = rule[0]
        non_default_matches = 0
        for idx, field in enumerate(candidate):
            if profile[idx] == field or field == '*':
                if field != '*':
                    non_default_matches += 1
                if idx == len(candidate)-1:
                    #all fields matched, adding as candidate
                    candidate_profiles.append((rule,non_default_matches))
            else:
                break
    
    def getKey(item):
        return item[1]

    sorted_profiles = sorted(candidate_profiles, key=getKey, reverse=True)
    print "CANDIDATE PROFILES"
    for candidate in sorted_profiles: print candidate

    print "BEST CANDIDATE"
    print sorted_profiles[0][0]
