# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'set_manager.ui'
#
# Created: Wed Mar  9 08:54:50 2016
#      by: PyQt5 UI code generator 5.2.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(400, 300)
        self.gridLayout_2 = QtWidgets.QGridLayout(Dialog)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_2 = QtWidgets.QLabel(Dialog)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout.addWidget(self.label_2)
        self.sets_combo = QtWidgets.QComboBox(Dialog)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.sets_combo.sizePolicy().hasHeightForWidth())
        self.sets_combo.setSizePolicy(sizePolicy)
        self.sets_combo.setMinimumSize(QtCore.QSize(0, 40))
        self.sets_combo.setMaximumSize(QtCore.QSize(16777215, 40))
        self.sets_combo.setObjectName("sets_combo")
        self.sets_combo.addItem("")
        self.sets_combo.addItem("")
        self.sets_combo.addItem("")
        self.horizontalLayout.addWidget(self.sets_combo)
        self.verticalLayout_3.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label = QtWidgets.QLabel(Dialog)
        self.label.setMaximumSize(QtCore.QSize(16777215, 40))
        self.label.setObjectName("label")
        self.verticalLayout_2.addWidget(self.label)
        self.configs_combo = QtWidgets.QComboBox(Dialog)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.configs_combo.sizePolicy().hasHeightForWidth())
        self.configs_combo.setSizePolicy(sizePolicy)
        self.configs_combo.setMinimumSize(QtCore.QSize(0, 40))
        self.configs_combo.setMaximumSize(QtCore.QSize(16777215, 40))
        self.configs_combo.setObjectName("configs_combo")
        self.configs_combo.addItem("")
        self.configs_combo.addItem("")
        self.configs_combo.addItem("")
        self.configs_combo.addItem("")
        self.configs_combo.addItem("")
        self.configs_combo.addItem("")
        self.verticalLayout_2.addWidget(self.configs_combo)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.remove_button = QtWidgets.QPushButton(Dialog)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.remove_button.sizePolicy().hasHeightForWidth())
        self.remove_button.setSizePolicy(sizePolicy)
        self.remove_button.setMinimumSize(QtCore.QSize(50, 50))
        self.remove_button.setMaximumSize(QtCore.QSize(50, 50))
        self.remove_button.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("list-remove.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.remove_button.setIcon(icon)
        self.remove_button.setAutoDefault(False)
        self.remove_button.setObjectName("remove_button")
        self.gridLayout.addWidget(self.remove_button, 0, 0, 1, 1)
        self.add_button = QtWidgets.QPushButton(Dialog)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.add_button.sizePolicy().hasHeightForWidth())
        self.add_button.setSizePolicy(sizePolicy)
        self.add_button.setMinimumSize(QtCore.QSize(50, 50))
        self.add_button.setMaximumSize(QtCore.QSize(50, 50))
        self.add_button.setText("")
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("list-add.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.add_button.setIcon(icon1)
        self.add_button.setAutoDefault(False)
        self.add_button.setObjectName("add_button")
        self.gridLayout.addWidget(self.add_button, 0, 1, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout)
        self.remove_all = QtWidgets.QPushButton(Dialog)
        self.remove_all.setMinimumSize(QtCore.QSize(0, 40))
        self.remove_all.setAutoDefault(False)
        self.remove_all.setObjectName("remove_all")
        self.verticalLayout.addWidget(self.remove_all)
        self.verticalLayout_2.addLayout(self.verticalLayout)
        self.horizontalLayout_2.addLayout(self.verticalLayout_2)
        self.configs_list = QtWidgets.QListView(Dialog)
        self.configs_list.setObjectName("configs_list")
        self.horizontalLayout_2.addWidget(self.configs_list)
        self.verticalLayout_3.addLayout(self.horizontalLayout_2)
        self.buttonBox = QtWidgets.QDialogButtonBox(Dialog)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.verticalLayout_3.addWidget(self.buttonBox)
        self.gridLayout_2.addLayout(self.verticalLayout_3, 0, 0, 1, 1)

        self.retranslateUi(Dialog)
        self.buttonBox.accepted.connect(Dialog.accept)
        self.buttonBox.rejected.connect(Dialog.reject)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.label_2.setText(_translate("Dialog", "Set :"))
        self.sets_combo.setItemText(0, _translate("Dialog", "Set1"))
        self.sets_combo.setItemText(1, _translate("Dialog", "Set2"))
        self.sets_combo.setItemText(2, _translate("Dialog", "Set3"))
        self.label.setText(_translate("Dialog", "Configurations of the set :"))
        self.configs_combo.setItemText(0, _translate("Dialog", "Config1"))
        self.configs_combo.setItemText(1, _translate("Dialog", "Config2"))
        self.configs_combo.setItemText(2, _translate("Dialog", "Config3"))
        self.configs_combo.setItemText(3, _translate("Dialog", "Config4"))
        self.configs_combo.setItemText(4, _translate("Dialog", "Config5"))
        self.configs_combo.setItemText(5, _translate("Dialog", "Config6"))
        self.remove_all.setText(_translate("Dialog", "Remove all"))

