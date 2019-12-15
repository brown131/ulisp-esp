 
const char LispLibrary[] PROGMEM = 
"(defun last (x) (nth (- (length x) 1) x))"
"(defun filter (fn lst) (mapcan (lambda (x) (when (fn x) (list x))) lst))"
"(defun remove (fn lst) (mapcan (lambda (x) (unless (fn x) (list x))) lst))"
"(defun reduce (fn acc lst) (if (not lst) acc (reduce fn (fn acc (car lst)) (cdr lst))))"
"(defun some (fn lst) (when lst (if (fn (car lst)) t (some fn (cdr lst)))))"
"(defun every (fn lst) (if (not lst) t (when (fn (car lst)) (every fn (cdr lst)))))"
"(defun take (n lst) (let ((fn (lambda (x lst acc) (if (minusp x) acc (fn (1- x) lst (cons (nth x lst) acc)))))) (fn (1- n) lst '())))"
"(defun drop (n lst) (if (zerop n) lst (drop (1- n) (cdr lst))))"
"(defun sprintln (x s) (princ x s) (princ \\#return s) (princ \\#newline s))"
"(defun connect-wifi () (wifi-connect \"deep13\" \"T@nkT0ps\"))"
"(defun show-file (fnm) (with-sd-card (s fnm) (loop (let ((l (read-line s))) (unless l (return nothing)) (princ l) (terpri)))))"
"(defun directory-tree (&optional (dir \"/\")) (dolist (d (list-directory dir)) (print d) (if (list-directory d) (directory-tree d))))"
; 
