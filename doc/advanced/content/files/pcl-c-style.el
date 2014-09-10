;;; pcl-c-style.el --- Pcl's C/C++ style for c-mode
;;
;; Keywords: c, tools
;;
;;;  pcl-c-style.el 
;;
;;; Software License Agreement (BSD License)
;;
;;  Point Cloud Library (PCL) - www.pointclouds.org
;;  Copyright (c) 2012, Willow Garage, Inc.
;;
;;  All rights reserved.
;;
;;  Redistribution and use in source and binary forms, with or without
;;  modification, are permitted provided that the following conditions
;;  are met:
;;
;;   * Redistributions of source code must retain the above copyright
;;     notice, this list of conditions and the following disclaimer.
;;   * Redistributions in binary form must reproduce the above
;;     copyright notice, this list of conditions and the following
;;     disclaimer in the documentation and/or other materials provided
;;     with the distribution.
;;   * Neither the name of Willow Garage, Inc. nor the names of its
;;     contributors may be used to endorse or promote products derived
;;     from this software without specific prior written permission.
;;
;;  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
;;  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
;;  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
;;  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
;;  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
;;  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
;;  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
;;  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
;;  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
;;  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
;;  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;  POSSIBILITY OF SUCH DAMAGE.
;;
;;; Commentary:
;;
;;  Provides the pcl C/C++ coding style. You may wish to add
;;  `pcl-set-c-style' to your `c-mode-common-hook' after requiring this
;;  file. For example:
;;
;;    (add-hook 'c-mode-common-hook 'pcl-set-c-style)
;;

;; Consider files with ".h" extension as c++
(setq auto-mode-alist (cons '("\\.h\\'" . c++-mode) auto-mode-alist))

;; Require cc-mode
(require 'cc-mode)

;; Define c++ access keys
(setq c-C++-access-key "\\<\\(slots\\|signals\\|private\\|protected\\|public\\)\\>[ \t]*[(slots\\|signals)]*[ \t]*:")

;; An inline function that return true if we are inside a class
(defsubst pcl-at-inclass-topmost-intro (sintax)
   (eq (caar sintax) 'inclass))


;; Indent after access keys if any
;; This code was adapted from here 
;; http://lists.gnu.org/archive/html/help-gnu-emacs/2009-09/msg00593.html
(defun pcl-indent-after-access-label-maybe (langelem)
  "Give an extra level of indentation to class members under an access
specifier, e.g.:

public
A(); <========== extra level of indentation.

This should really be properly implemented in CC Mode, but it's not."
  (and
   (let (m-type)
     (when
	 (save-excursion
	   (back-to-indentation)
	   ;; Go back one "statement" at a time till we reach a label or
	   ;; something which isn't an inclass-topmost-intro
	   (while
	       (and (eq (setq m-type (c-beginning-of-statement-1)) 'previous)
		    (pcl-at-inclass-topmost-intro (c-guess-basic-syntax))))
	   ;; Have we found "private:", "public": or "protected"?
	   (and (eq m-type 'label)
		(looking-at c-C++-access-key)))
       (save-excursion
	 (back-to-indentation)
	 c-basic-offset)))))

(defconst pcl-c-style
  `("gnu"
    (c-recognize-knr-p . nil)
    (c-enable-xemacs-performance-kludge-p . t) ; speed up indentation in XEmacs
    (c-basic-offset . 2)
;;    (c-tab-always-indent        . t)
    (c-comment-only-line-offset . 0)
    (c-hanging-braces-alist . ((defun-open before)
			       (defun-open after)
			       (namespace-open before)
			       (namespace-open after)))
    (c-hanging-colons-alist . ((member-init-intro before)
			       (inher-intro)))
    (c-cleanup-list         . (scope-operator
			       empty-defun-braces
			       defun-close-semi
			       space-before-funcall))
    (c-offsets-alist
     (inline-open . +)
     (inclass . +)
     (access-label . 0)
     (topmost-intro . pcl-indent-after-access-label-maybe)
     (topmost-intro-cont 0)
     (statement-cont . 0)
     (substatement-open . 0)
     (substatement-label . 0)
     (statement-case-open . 0)
     (case-label . +)
     ))
  "PCL C/C++ Programming Style")

(defun pcl-set-c-style ()
  "Set the current buffer's c-style to PCL C/C++ Programming
   Style. Meant to be added to `c-mode-common-hook'."
  (interactive)
  (setq tab-width 2
        ;; this will make sure spaces are used instead of tabs
        indent-tabs-mode nil)
  (c-add-style "PCL" pcl-c-style t)
  (c-toggle-auto-newline 1)
  (c-toggle-hungry-state 1))

;; Switch from *.<impl> to *.<head> and vice versa
(defun switch-cpp-or-hpp-to-h ()
  (interactive)
  (when (string-match "^\\(.*\\)/\\(.*\\)\\.\\([^.]*\\)$" buffer-file-name)
    (let ((path (match-string 1 buffer-file-name))
	  (name (match-string 2 buffer-file-name))
	  (suffix (match-string 3 buffer-file-name)))
      (cond ((string-match suffix "cpp")
	     (when (string-match "^\\(.*\\)/\\(.*\\)/src$" path)
	       (let ((pcl-root (match-string 1 path))
		     (subsys (match-string 2 path)))
		 (if (file-exists-p (concat pcl-root "/" subsys "/include/pcl/" subsys "/" name ".h"))
		     (find-file (concat pcl-root "/" subsys "/include/pcl/" subsys "/" name ".h")))))
	     (cond ((file-exists-p (concat name ".hpp"))
		    (find-file (concat name ".hpp"))
		    )))
	    ((string-match suffix "h")
	     (message "in h")
	     (cond ((file-exists-p (concat path "/impl/" name ".hpp"))
		    (find-file (concat path "/impl/" name ".hpp"))
		    )
		   ((file-exists-p (concat name ".cpp"))
		    (find-file (concat name ".cpp"))
		    )))
	    ((string-match suffix "hpp")
	     (message "in hpp")
	     (cond ((file-exists-p (concat path "/../" name ".h"))
	    	    (find-file (concat path "/../" name ".h"))
	    	    )
	    	   ))
	     ))))

(provide 'pcl-c-style)
;;; pcl-c-style.el
