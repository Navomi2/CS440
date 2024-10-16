"""
Part 1: Simple baseline that only uses word statistics to predict tags
"""

def baseline(train, test):
    '''
    input:  training data (list of sentences, with tags on the words). E.g.,  [[(word1, tag1), (word2, tag2)], [(word3, tag3), (word4, tag4)]]
            test data (list of sentences, no tags on the words). E.g.,  [[word1, word2], [word3, word4]]
    output: list of sentences, each sentence is a list of (word,tag) pairs.
            E.g., [[(word1, tag1), (word2, tag2)], [(word3, tag3), (word4, tag4)]]
    '''
#     print(f"train: {train}")
# need to map cross product of tag and word to a count -> nested dict word: tag: count
    tag_to_count = {}
    word_to_tag_to_count = {}
    for sentence in train:
        for (word, tag) in sentence:
                if tag in tag_to_count:
                        tag_to_count[tag] += 1
                else:
                        tag_to_count[tag] = 1

                if word in word_to_tag_to_count:
                        if tag in word_to_tag_to_count[word]:
                                word_to_tag_to_count[word][tag] += 1
                        else:
                                word_to_tag_to_count[word][tag] = 1
                else:
                        word_to_tag_to_count[word] = {}
                        word_to_tag_to_count[word][tag] = 1

# find most common tag to use for unseen words
    sorted_tags = sorted(tag_to_count.items(), key=lambda x:x[1], reverse=True) #sort by count
    most_common_tag = sorted_tags[0][0]
    for (tag, count) in sorted_tags:
        if tag != 'START' and tag != 'END': #to exclude tags like START or END
              most_common_tag = tag
              break  

#     print(f"most_common_tag: {most_common_tag}")

    out = []
    for sentence in test:
        # print(f"sentence: {sentence}")
        sentence_list = []
        for word in sentence:
                if word in word_to_tag_to_count:
                        sorted_tag_to_count = sorted(word_to_tag_to_count[word].items(), key=lambda x:x[1], reverse=True) #sort by count
                        tag = sorted_tag_to_count[0][0]
                        sentence_list.append((word, tag))
                else:
                       sentence_list.append((word, most_common_tag)) 

        out.append(sentence_list)
#     print(f"out: {out}")
    return out