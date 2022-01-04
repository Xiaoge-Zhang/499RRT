import numpy as np


class tree:
    def __init__(self,value):
        self.parent = None
        self.child = []
        self.value = value

    def add_node(self,Node):
        Node.parent=self
        self.child.append(Node)

    def travel(self):
        value_list=[self.value]
        if self.child is not None:
            for node in self.child:
                value_list.extend(node.travel())
        return value_list

    def search(self, target):

        if np.array_equal(np.array(self.value), np.array(target)):
            return self
        else:
            if len(self.child) != 0:
                for sub_child in self.child:
                    if not sub_child.search(target):
                        continue
                    return sub_child.search(target)
            else:
                return False


if __name__ == '__main__':
    tree1=tree(1)
    tree2=tree(2)
    tree3=tree(3)
    tree4 = tree(4)
    tree5 = tree(5)
    tree6 = tree(6)
    tree7 = tree(7)
    tree1.add_node(tree2)
    tree1.add_node(tree3)
    tree2.add_node(tree7)
    tree3.add_node(tree4)
    tree3.add_node(tree5)
    tree3.add_node(tree6)
    a=tree1.travel()
    print(tree1.search(3).value)