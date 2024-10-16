"""
Part 2: This is the simplest version of viterbi that doesn't do anything special for unseen words
but it should do better than the baseline at words with multiple tags (because now you're using context
to predict the tag).
"""

import math
from collections import defaultdict, Counter
from math import log

# Note: remember to use these two elements when you find a probability is 0 in the training data.
epsilon_for_pt = 1e-5
emit_epsilon = 1e-5   # exact setting seems to have little or no effect
Nt = {}

def training(sentences):
    """
    Computes initial tags, emission words and transition tag-to-tag probabilities
    :param sentences:
    :return: intitial tag probs, emission words given tag probs, transition of tags to tags probs
    """
    # print(sentences)
    init_prob = defaultdict(lambda: 0) # {init tag: #}
    emit_prob = defaultdict(lambda: defaultdict(lambda: 0)) # {tag: {word: # }}
    trans_prob = defaultdict(lambda: defaultdict(lambda: 0)) # {tag0:{tag1: # }}
    
    # TODO: (I)
    # Input the training set, output the formatted probabilities according to data statistics.
    laplace = epsilon_for_pt
    tag_to_count = {}

    num_starting_words = 0
    for sentence in sentences:
        tag0 = None
        for i in range(len(sentence)):
            (word,tag) = sentence[i]
            if tag in tag_to_count:
                tag_to_count[tag] += 1
            else:
                tag_to_count[tag] = 1

            if i == 0: #skip START when checking first word of sentence?? IDK IF THIS WORKS FOR OTHER TAGGING SYSTEMS
                init_prob[tag] += 1
                num_starting_words += 1
                tag0 = tag
            else:
                trans_prob[tag0][tag] += 1
            emit_prob[tag][word] += 1
            tag0 = tag

    # use laplace smoothing
    num_tags = len(emit_prob)
    global Nt
    Nt = tag_to_count
    for tag in emit_prob:
        Vt = len(emit_prob[tag]) # number of unique words for tag
        nt = Nt[tag] # number of words in training data for tag
        for word in emit_prob[tag]:
            emit_prob[tag][word] = (emit_prob[tag][word] + laplace) / (nt + laplace*(Vt+1))

    for tag in init_prob:
        Vt = init_prob[tag]
        nt = Nt[tag]
        init_prob[tag] = (init_prob[tag] + laplace) / (nt + laplace*(Vt+1))

    for tag0 in trans_prob:
        Vt = len(trans_prob[tag0])
        nt = Nt[tag0]
        for tag1 in trans_prob[tag0]:
            trans_prob[tag0][tag1] = (trans_prob[tag0][tag1] + laplace) / (nt + laplace*(Vt+1))

    # print(f"init_prob: {init_prob}")
    # print(f"trans_prob: {trans_prob}")
    # print(f"emit_prob: {emit_prob}")
    
    return init_prob, emit_prob, trans_prob

def viterbi_stepforward(i, word, prev_prob, prev_predict_tag_seq, emit_prob, trans_prob):
    """
    Does one step of the viterbi function
    :param i: The i'th column of the lattice/MDP (0-indexing)
    :param word: The i'th observed word
    :param prev_prob: A dictionary of tags to probs representing the max probability of getting to each tag at in the
    previous column of the lattice
    :param prev_predict_tag_seq: A dictionary representing the predicted tag sequences leading up to the previous column
    of the lattice for each tag in the previous column
    :param emit_prob: Emission probabilities
    :param trans_prob: Transition probabilities
    :return: Current best log probs leading to the i'th column for each tag, and the respective predicted tag sequences
    """
    # print(i, word)
    log_prob = {} # This should store the log_prob for all the tags at current column (i)
    predict_tag_seq = {} # This should store the tag sequence to reach each tag at column (i)
    # TODO: (II)
    # implement one step of trellis computation at column (i)
    # You should pay attention to the i=0 special case.
    log_prob = prev_prob
    # print(log_prob)
    b = {}
    laplace = epsilon_for_pt
   
    if i == 0: #prev_prob is init_prob
        for t in emit_prob:
            if word in emit_prob[t]:  
                log_prob[t] += math.log(emit_prob[t][word])
            else:
                Vt = len(emit_prob[t])
                nt = Nt[t]
                log_prob[t] = (laplace) / (nt + laplace*(Vt+1))
    else:
        for tagb in prev_prob: #for every tagb
            prob_w_taga = 0 #current prob

            # log_e_prob = epsilon_for_pt
            # if emit_prob[tagb][word] > 0:
            #     log_e_prob = math.log(emit_prob[tagb][word])
            if word in emit_prob[tagb]:  
                log_prob[tagb] += math.log(emit_prob[tagb][word])
            else:
                Vt = len(emit_prob[tagb])
                log_prob[tagb] = (laplace) / (Nt[tagb] + laplace*(Vt+1))

            for taga in prev_prob:
                log_t_prob = log(epsilon_for_pt)
                if trans_prob[taga][tagb] > 0:
                    log_t_prob = math.log(trans_prob[taga][tagb]) #does this go taga->tag?
                prob_w_taga += log_t_prob
                prob_w_taga += log_prob[tagb]

                if prob_w_taga > log_prob[tagb]:
                    log_prob[tagb] = prob_w_taga
                    b[tagb] = taga

    predict_tag_seq = prev_predict_tag_seq
    for tag in predict_tag_seq:
        sequence = predict_tag_seq[tag]
        if sequence:
            sequence_end = sequence[len(sequence) - 1]
            predict_tag_seq[tag].append(b[sequence_end])

    # print(f"log prob: {log_prob}")
    # print(f"predict tag seq: {predict_tag_seq}")
    return log_prob, predict_tag_seq

def viterbi_1(train, test, get_probs=training):
    '''
    input:  training data (list of sentences, with tags on the words). E.g.,  [[(word1, tag1), (word2, tag2)], [(word3, tag3), (word4, tag4)]]
            test data (list of sentences, no tags on the words). E.g.,  [[word1, word2], [word3, word4]]
    output: list of sentences, each sentence is a list of (word,tag) pairs.
            E.g., [[(word1, tag1), (word2, tag2)], [(word3, tag3), (word4, tag4)]]
    '''
    init_prob, emit_prob, trans_prob = get_probs(train)
    
    predicts = []
    # print(init_prob)
    for sen in range(len(test)):
        sentence=test[sen]
        length = len(sentence)
        log_prob = {}
        predict_tag_seq = {}
        # init log prob
        for t in emit_prob:
            if t in init_prob:
                log_prob[t] = log(init_prob[t])
            else:
                log_prob[t] = log(epsilon_for_pt)
            predict_tag_seq[t] = []

        # forward steps to calculate log probs for sentence
        for i in range(length):
            log_prob, predict_tag_seq = viterbi_stepforward(i, sentence[i], log_prob, predict_tag_seq, emit_prob,trans_prob)
            
        # TODO:(III) 
        # according to the storage of probabilities and sequences, get the final prediction.
        sentence_prediction = []

        max_tag = max(log_prob, key=log_prob.get)
        
        tag_sequence = predict_tag_seq[max_tag]

        if tag_sequence:
            tag_idx = 0
            for word in sentence:
                sentence_prediction.append((word, tag_sequence[tag_idx]))
                tag_idx += 1
                
            predicts.append(sentence_prediction)

    # print(f"predicts: {predicts}")
    return predicts